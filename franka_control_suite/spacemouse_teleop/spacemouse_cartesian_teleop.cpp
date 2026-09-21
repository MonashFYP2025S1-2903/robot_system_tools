// SpaceMouse -> Franka Panda Cartesian velocity teleop (libfranka only, no ROS, no Isaac Lab).
// Reads the 3Dconnexion SpaceMouse straight from its hidraw node and streams a
// rate-limited, workspace-boxed Cartesian velocity command over FCI.
//
// Modes: --dry-run (no robot, prints what would be commanded), --status (read robot pose only),
// default = drive the robot.

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>

#include <franka/duration.h>
#include <Eigen/Dense>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/model.h>
#include <franka/robot.h>

namespace {

constexpr double kRawMax = 350.0;
constexpr double kBoxTaper = 0.03;  // m, speed ramps to zero over this distance before a box wall
constexpr double kMaxVLimit = 0.20;
constexpr double kMaxWLimit = 0.50;
constexpr double kQMin[7] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr double kQMax[7] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
constexpr double kJointStop = 0.10;   // rad from a limit where motion toward it is fully stopped
constexpr double kJointTaper = 0.25;  // rad over which motion toward a limit slows first

std::atomic<bool> g_stop{false};
void OnSignal(int) { g_stop = true; }

int64_t NowNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

struct Shared {
  std::atomic<int> raw[6]{};
  std::atomic<int64_t> last_ns{0};
  std::atomic<bool> dead{false};
  std::atomic<int> battery{-1};
  std::atomic<int> btn{0};       // report id 3, byte 1: bitmask of currently pressed buttons
  std::atomic<int> btn2{0};      // report id 3, byte 2
  std::atomic<int> other_id{0};  // last report id that is none of the known ones
  std::atomic<double> grip_target{-1.0};  // commanded hand opening, m (the gripper action)
  std::atomic<double> grip_w{-1.0};       // measured finger opening, m
  std::atomic<bool> grip_grasped{false};  // fingers stopped on an object and are holding it
  std::atomic<double> mon[14]{};  // 0-2 user lin, 3-5 user ang, 6-8 base lin out, 9-11 pos - home, 12 joint-limit scale
};

struct Cfg {
  std::string ip = "172.16.0.2";
  std::string device;
  bool dry_run = false;
  bool status = false;
  bool rot = false;
  bool realtime = true;
  bool recover = false;
  double vmax = 0.05;      // m/s at full deflection
  double wmax = 0.15;      // rad/s at full deflection
  double amax = 0.5;       // m/s^2 (angular limit is 2x this in rad/s^2)
  double yaw_deg = 0.0;    // direction the user's "forward" points, in the robot base frame
  double box_xy = 0.20;    // m, half-extent around the start pose
  double box_z_up = 0.20;
  double box_z_down = 0.15;
  bool gripper = true;
  bool gripper_home = false;  // run the hand's homing routine (moves the fingers) at start
  double grip_force = 10.0;   // N, hold force when closing on an object
  double grip_speed = 0.05;   // m/s, finger speed
  double grip_rate = 0.04;    // m/s, how fast the target width changes while a button is held
  double rs[3] = {1.0, 1.0, 1.0};  // sign multipliers: cap rotation about forward / left / up
  bool abs_box = false;    // absolute box in the robot base frame instead of one around the start pose
  double abs_lo[3]{};
  double abs_hi[3]{};
  double max_s = 300.0;   // hard time limit, 0 = none
  double stale_s = 0.5;    // zero the command when no HID report arrived for this long
  int deadzone = 20;       // raw counts out of ~350
};

std::string ReadFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

// The motion interface is the hidraw node of vendor 0x256F whose report descriptor opens with
// Usage Page (Generic Desktop) / Usage (Multi-axis Controller): 05 01 09 08.
std::string FindHidraw() {
  DIR* d = opendir("/sys/class/hidraw");
  if (!d) return "";
  std::string found;
  while (dirent* e = readdir(d)) {
    std::string name = e->d_name;
    if (name.rfind("hidraw", 0) != 0) continue;
    std::string base = "/sys/class/hidraw/" + name + "/device/";
    if (ReadFile(base + "uevent").find("HID_ID=0003:0000256F:") == std::string::npos) continue;
    std::string rd = ReadFile(base + "report_descriptor");
    if (rd.size() >= 4 && (uint8_t)rd[0] == 0x05 && (uint8_t)rd[1] == 0x01 &&
        (uint8_t)rd[2] == 0x09 && (uint8_t)rd[3] == 0x08) {
      found = "/dev/" + name;
      break;
    }
  }
  closedir(d);
  return found;
}

void ReaderLoop(int fd, Shared& s) {
  uint8_t buf[64];
  auto i16 = [&](int o) { return (int)(int16_t)(buf[o] | (buf[o + 1] << 8)); };
  while (!g_stop) {
    pollfd p{fd, POLLIN, 0};
    int r = poll(&p, 1, 100);
    if (r < 0) {
      if (errno == EINTR) continue;
      s.dead = true;
      return;
    }
    if (r == 0) continue;
    if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      s.dead = true;
      return;
    }
    ssize_t n = read(fd, buf, sizeof buf);
    if (n <= 0) {
      s.dead = true;
      return;
    }
    if (buf[0] == 1 && n >= 13) {
      for (int k = 0; k < 6; ++k) s.raw[k] = i16(1 + 2 * k);
      s.last_ns = NowNs();
    } else if (buf[0] == 1 && n >= 7) {
      for (int k = 0; k < 3; ++k) s.raw[k] = i16(1 + 2 * k);
      s.last_ns = NowNs();
    } else if (buf[0] == 2 && n >= 7) {
      for (int k = 0; k < 3; ++k) s.raw[3 + k] = i16(1 + 2 * k);
      s.last_ns = NowNs();
    } else if (buf[0] == 23 && n >= 2) {
      s.battery = buf[1];
    } else if (buf[0] == 3 && n >= 2) {
      s.btn = buf[1];
      s.btn2 = n > 2 ? buf[2] : 0;
    } else {
      s.other_id = buf[0];
    }
  }
}

double Shape(int raw, int deadzone) {
  double a = std::abs(raw);
  if (a <= deadzone) return 0.0;
  double x = std::min((a - deadzone) / (kRawMax - deadzone), 1.0);
  return std::copysign(x * x, (double)raw);
}

// Estimates joint velocities for a Cartesian command (damped least squares) and returns a 0..1
// scale that slows and then stops motion toward any joint limit.
double JointLimitScale(const franka::Model& model, const franka::RobotState& st, const double v[6]) {
  std::array<double, 42> ja = model.zeroJacobian(franka::Frame::kEndEffector, st);
  Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(ja.data());
  Eigen::Matrix<double, 6, 1> vel;
  for (int i = 0; i < 6; ++i) vel[i] = v[i];
  Eigen::Matrix<double, 6, 6> A = J * J.transpose() + 1e-4 * Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 7, 1> qd = J.transpose() * A.ldlt().solve(vel);
  double scale = 1.0;
  for (int i = 0; i < 7; ++i) {
    double room;
    if (qd[i] > 1e-6) room = kQMax[i] - st.q[i];
    else if (qd[i] < -1e-6) room = st.q[i] - kQMin[i];
    else continue;
    scale = std::min(scale, std::clamp((room - kJointStop) / kJointTaper, 0.0, 1.0));
  }
  return scale;
}

struct Teleop {
  const Cfg& c;
  Shared& s;
  double v[6]{};
  double home[3]{};
  double lo[3]{};
  double hi[3]{};
  bool have_home = false;
  bool armed = false;
  bool tilt_mode = false;
  int prev_btn = 0;
  double center_s = 0.0;

  Teleop(const Cfg& cfg, Shared& sh) : c(cfg), s(sh) {}

  void SetHome(const double p[3]) {
    for (int i = 0; i < 3; ++i) {
      home[i] = p[i];
      lo[i] = p[i] - (i < 2 ? c.box_xy : c.box_z_down);
      hi[i] = p[i] + (i < 2 ? c.box_xy : c.box_z_up);
    }
    have_home = true;
  }

  void SetAbsBox() {
    for (int i = 0; i < 3; ++i) {
      home[i] = 0.0;
      lo[i] = c.abs_lo[i];
      hi[i] = c.abs_hi[i];
    }
    have_home = true;
  }

  bool AtRest() const {
    for (double x : v)
      if (x != 0.0) return false;
    return true;
  }

  double BoxScale(int i, const double pos[3], double vi) const {
    if (!have_home || vi == 0.0) return 1.0;
    double room = vi > 0 ? hi[i] - pos[i] : pos[i] - lo[i];
    return std::clamp(room / kBoxTaper, 0.0, 1.0);
  }

  void Step(double dt, const double pos[3], bool stopping, double out[6]) {
    int64_t last = s.last_ns.load();
    bool stale = last == 0 || (NowNs() - last) > (int64_t)(c.stale_s * 1e9);
    double n[6];
    bool centered = true;
    for (int k = 0; k < 6; ++k) {
      n[k] = Shape(stale ? 0 : s.raw[k].load(), c.deadzone);
      if (n[k] != 0.0) centered = false;
    }
    if (!armed) {
      center_s = centered ? center_s + dt : 0.0;
      if (center_s > 0.3) armed = true;
    }

    int b = s.btn.load();
    if (c.rot && b == 3 && prev_btn != 3) tilt_mode = !tilt_mode;  // both side buttons toggle
    prev_btn = b;

    // Device axes -> user frame (forward, left, up), from the dry-run calibration of 2026-09-21.
    // Angular part is the cap's rotation vector (right-hand rule) about forward, left, up.
    double ul[3] = {-n[1], -n[0], -n[2]};
    double ua[3] = {-n[4] * c.rs[0], -n[3] * c.rs[1], -n[5] * c.rs[2]};
    if (!armed || stopping || !c.rot) {
      for (double& x : ua) x = 0.0;
    }
    if (!armed || stopping) {
      for (double& x : ul) x = 0.0;
    }
    // Tilts read like pushes on this device, so tilts only count in tilt mode (which drops
    // translation); twist (yaw) always counts. Translation and rotation are mutually exclusive.
    if (tilt_mode) {
      for (double& x : ul) x = 0.0;
    } else {
      ua[0] = ua[1] = 0.0;
    }
    if (c.rot) {
      double mt = std::sqrt(ul[0] * ul[0] + ul[1] * ul[1] + ul[2] * ul[2]);
      double mr = std::sqrt(ua[0] * ua[0] + ua[1] * ua[1] + ua[2] * ua[2]);
      if (mr > mt) {
        for (double& x : ul) x = 0.0;
      } else {
        for (double& x : ua) x = 0.0;
      }
    }
    s.mon[13] = tilt_mode ? 1.0 : 0.0;
    double ml = std::sqrt(ul[0] * ul[0] + ul[1] * ul[1] + ul[2] * ul[2]);
    if (ml > 1.0)
      for (double& x : ul) x /= ml;
    double ma = std::sqrt(ua[0] * ua[0] + ua[1] * ua[1] + ua[2] * ua[2]);
    if (ma > 1.0)
      for (double& x : ua) x /= ma;

    double yaw = c.yaw_deg * M_PI / 180.0, cy = std::cos(yaw), sy = std::sin(yaw);
    double tgt[6] = {c.vmax * (ul[0] * cy - ul[1] * sy), c.vmax * (ul[0] * sy + ul[1] * cy),
                     c.vmax * ul[2],
                     c.wmax * (ua[0] * cy - ua[1] * sy), c.wmax * (ua[0] * sy + ua[1] * cy),
                     c.wmax * ua[2]};
    for (int i = 0; i < 6; ++i) {
      double lim = (i < 3 ? c.amax : 2.0 * c.amax) * dt;
      v[i] += std::clamp(tgt[i] - v[i], -lim, lim);
    }
    for (int i = 0; i < 3; ++i) out[i] = v[i] * BoxScale(i, pos, v[i]);
    for (int i = 3; i < 6; ++i) out[i] = v[i];

    for (int i = 0; i < 3; ++i) {
      s.mon[i] = ul[i];
      s.mon[3 + i] = ua[i];
      s.mon[6 + i] = out[i];
      s.mon[9 + i] = have_home ? pos[i] - home[i] : 0.0;
    }
  }
};

// Gripper: hold the left button to close, the right button to open (both together is the
// tilt-mode toggle and is ignored). One long move runs while a button is held and stop() ends it
// on release, so the fingers move smoothly. If the fingers stall against something while closing
// they switch to a hold at grip_force. dry = simulate the hand instead of connecting to it.
void GripperLoop(const Cfg& c, Shared& s, bool dry) {
  enum State { kIdle, kClosing, kOpening, kHolding };
  std::atomic<int> cmd{0};  // 1 = move to closed, 2 = move to open, 3 = grasp at grasp_w
  std::atomic<bool> busy{false};
  std::atomic<bool> quit{false};
  std::atomic<double> grasp_w{0.0};
  double max_w = 0.08;
  std::unique_ptr<franka::Gripper> g;
  std::thread worker;
  try {
    double w = 0.08;
    if (!dry) {
      g = std::make_unique<franka::Gripper>(c.ip);
      if (c.gripper_home) g->homing();
      franka::GripperState gs = g->readOnce();
      w = gs.width;
      if (gs.max_width > 0.01) max_w = gs.max_width;
      worker = std::thread([&] {
        while (!quit) {
          int k = cmd.exchange(0);
          if (k == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
          }
          try {
            if (k == 1) g->move(0.0, c.grip_speed);
            else if (k == 2) g->move(max_w, c.grip_speed);
            else if (k == 3) g->grasp(grasp_w, c.grip_speed, c.grip_force, 0.05, 0.05);
          } catch (const franka::CommandException&) {
          } catch (const std::exception& e) {
            std::fprintf(stderr, "gripper: %s\n", e.what());
          }
          busy = false;
        }
      });
    }
    auto stop_hand = [&]() {
      try {
        g->stop();
      } catch (const std::exception&) {
      }
      for (int i = 0; i < 500 && busy; ++i) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    };
    auto start = [&](int k) {
      busy = true;
      cmd = k;
    };

    State st = kIdle;
    int prev_b = 0;
    double held = 0.0, since_start = 0.0, stall_t = 0.0, last_w = w, w_start = w;
    bool moved = false;
    s.grip_w = w;
    s.grip_target = w;
    auto t_prev = std::chrono::steady_clock::now();
    while (!g_stop) {
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - t_prev).count();
      t_prev = now;
      int b = s.btn.load();
      held = ((b == 1 || b == 2) && b == prev_b) ? held + dt : 0.0;
      prev_b = b;
      bool left = held > 0.10 && b == 1, right = held > 0.10 && b == 2;
      if (!dry) w = g->readOnce().width;

      auto begin_move = [&](State next, int k) {
        st = next;
        moved = false;
        since_start = 0.0;
        stall_t = 0.0;
        w_start = last_w = w;
        if (!dry) start(k);
      };
      switch (st) {
        case kIdle:
          if (left && w > 0.002) begin_move(kClosing, 1);
          else if (right && w < max_w - 0.002) begin_move(kOpening, 2);
          break;
        case kClosing:
          since_start += dt;
          if (dry) w = std::max(0.0, w - c.grip_speed * dt);
          if (!left) {
            if (!dry) stop_hand();
            st = kIdle;
          } else if (dry) {
            if (w <= 0.0) st = kIdle;
          } else {
            if (std::abs(w - w_start) > 0.001) moved = true;
            stall_t = std::abs(w - last_w) < 0.0003 ? stall_t + dt : 0.0;
            last_w = w;
            bool stalled = (moved && stall_t > 0.15) || (!moved && since_start > 0.5);
            if (stalled && w > 0.003) {  // something is between the fingers: hold it
              stop_hand();
              grasp_w = w;
              start(3);
              st = kHolding;
            } else if (!busy) {
              st = kIdle;  // reached fully closed
            }
          }
          break;
        case kOpening:
          if (dry) w = std::min(max_w, w + c.grip_speed * dt);
          if (!right) {
            if (!dry) stop_hand();
            st = kIdle;
          } else if (dry ? w >= max_w : !busy) {
            st = kIdle;  // fully open
          }
          break;
        case kHolding:
          if (right) begin_move(kOpening, 2);
          break;
      }
      s.grip_target = st == kClosing ? 0.0 : (st == kOpening ? max_w : w);
      s.grip_w = w;
      s.grip_grasped = st == kHolding;
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (!dry && busy) stop_hand();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "gripper unavailable: %s\n", e.what());
  }
  quit = true;
  if (worker.joinable()) worker.join();
}

void MonitorLoop(Shared& s, Teleop& tp) {
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    int64_t last = s.last_ns.load();
    double age = last == 0 ? -1.0 : (NowNs() - last) / 1e9;
    std::printf(
        "raw=(%4d %4d %4d %4d %4d %4d) user[fwd left up]=(%+.2f %+.2f %+.2f) "
        "ang=(%+.2f %+.2f %+.2f) v_base=(%+.3f %+.3f %+.3f) pos_rel=(%+.3f %+.3f %+.3f) "
        "armed=%d jl=%.2f mode=%s grip_target=%.3f w=%.3f hold=%d btn=%d,%d oid=%d age=%.2fs\n",
        s.raw[0].load(), s.raw[1].load(), s.raw[2].load(), s.raw[3].load(), s.raw[4].load(),
        s.raw[5].load(), s.mon[0].load(), s.mon[1].load(), s.mon[2].load(), s.mon[3].load(),
        s.mon[4].load(), s.mon[5].load(), s.mon[6].load(), s.mon[7].load(), s.mon[8].load(),
        s.mon[9].load(), s.mon[10].load(), s.mon[11].load(), (int)tp.armed, s.mon[12].load(), s.mon[13].load() > 0.5 ? "TILT" : "move",
        s.grip_target.load(), s.grip_w.load(), (int)s.grip_grasped.load(), s.btn.load(),
        s.btn2.load(), s.other_id.load(), age);
    std::fflush(stdout);
  }
}

bool ParseArgs(int argc, char** argv, Cfg& c) {
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
    auto num = [&](double& dst) {
      const char* v = next();
      if (!v) return false;
      dst = std::atof(v);
      return true;
    };
    bool ok = true;
    if (a == "--ip") {
      const char* v = next();
      ok = v;
      if (v) c.ip = v;
    } else if (a == "--device") {
      const char* v = next();
      ok = v;
      if (v) c.device = v;
    } else if (a == "--dry-run") c.dry_run = true;
    else if (a == "--status") c.status = true;
    else if (a == "--rot") c.rot = true;
    else if (a == "--no-rt") c.realtime = false;
    else if (a == "--recover") c.recover = true;
    else if (a == "--no-gripper") c.gripper = false;
    else if (a == "--gripper-home") c.gripper_home = true;
    else if (a == "--grip-force") ok = num(c.grip_force);
    else if (a == "--grip-speed") ok = num(c.grip_speed);
    else if (a == "--grip-rate") ok = num(c.grip_rate);
    else if (a == "--rot-signs") {
      for (int k = 0; k < 3 && ok; ++k) ok = num(c.rs[k]);
    }
    else if (a == "--abs-box") {
      double* dst[6] = {&c.abs_lo[0], &c.abs_hi[0], &c.abs_lo[1],
                        &c.abs_hi[1], &c.abs_lo[2], &c.abs_hi[2]};
      for (int k = 0; k < 6 && ok; ++k) ok = num(*dst[k]);
      c.abs_box = ok;
    }
    else if (a == "--vmax") ok = num(c.vmax);
    else if (a == "--wmax") ok = num(c.wmax);
    else if (a == "--amax") ok = num(c.amax);
    else if (a == "--yaw-deg") ok = num(c.yaw_deg);
    else if (a == "--box-xy") ok = num(c.box_xy);
    else if (a == "--box-z-up") ok = num(c.box_z_up);
    else if (a == "--box-z-down") ok = num(c.box_z_down);
    else if (a == "--max-seconds") ok = num(c.max_s);
    else if (a == "--stale-s") ok = num(c.stale_s);
    else if (a == "--deadzone") {
      double d = c.deadzone;
      ok = num(d);
      c.deadzone = (int)d;
    } else ok = false;
    if (!ok) {
      std::fprintf(stderr, "bad or unknown argument: %s\n", a.c_str());
      return false;
    }
  }
  if (c.grip_force < 1.0 || c.grip_force > 40.0 || c.grip_speed <= 0 || c.grip_speed > 0.10 ||
      c.grip_rate <= 0 || c.grip_rate > 0.10) {
    std::fprintf(stderr,
                 "refusing: need 1 <= grip-force <= 40 N, 0 < grip-speed <= 0.10 m/s, "
                 "0 < grip-rate <= 0.10 m/s\n");
    return false;
  }
  if (c.vmax <= 0 || c.vmax > kMaxVLimit || c.wmax <= 0 || c.wmax > kMaxWLimit || c.amax <= 0) {
    std::fprintf(stderr, "refusing: need 0 < vmax <= %.2f m/s, 0 < wmax <= %.2f rad/s, amax > 0\n",
                 kMaxVLimit, kMaxWLimit);
    return false;
  }
  return true;
}

int RunStatus(const Cfg& c) {
  franka::Robot robot(c.ip, franka::RealtimeConfig::kIgnore);
  franka::RobotState st = robot.readOnce();
  std::printf("connected to %s, libfranka server version %u\n", c.ip.c_str(),
              (unsigned)robot.serverVersion());
  static const char* kModes[] = {"Other", "Idle", "Move", "Guiding", "Reflex", "UserStopped",
                                 "AutomaticErrorRecovery"};
  int m = (int)st.robot_mode;
  std::printf("robot_mode=%d (%s)  EE position (base frame) = (%.4f %.4f %.4f) m\n", m,
              m >= 0 && m < 7 ? kModes[m] : "?", st.O_T_EE[12], st.O_T_EE[13], st.O_T_EE[14]);
  double tilt = std::acos(std::clamp(-st.O_T_EE[10], -1.0, 1.0)) * 180.0 / M_PI;
  std::printf("gripper axis in base frame = (%+.2f %+.2f %+.2f), %.0f deg away from straight down\n",
              st.O_T_EE[8], st.O_T_EE[9], st.O_T_EE[10], tilt);
  for (int i = 0; i < 7; ++i)
    std::printf("  q%d = %+.3f rad   margin to lower %.3f, to upper %.3f\n", i + 1, st.q[i],
                st.q[i] - kQMin[i], kQMax[i] - st.q[i]);
  try {
    franka::Gripper g(c.ip);
    franka::GripperState gs = g.readOnce();
    std::printf("gripper: opening %.3f m, max opening %.3f m, holding an object: %s\n", gs.width,
                gs.max_width, gs.is_grasped ? "yes" : "no");
  } catch (const std::exception& e) {
    std::printf("gripper unavailable: %s\n", e.what());
  }
  franka::Model model = robot.loadModel();
  const char* dirs[6] = {"+x (away from base)", "-x (toward base)", "+y (robot left)",
                         "-y (robot right)", "+z (up)", "-z (down)"};
  std::printf("joint-limit guard scale per direction (1.00 = free, 0.00 = blocked):\n");
  for (int d = 0; d < 6; ++d) {
    double v[6] = {0, 0, 0, 0, 0, 0};
    v[d / 2] = (d % 2 == 0) ? 0.05 : -0.05;
    std::printf("  %-20s %.2f\n", dirs[d], JointLimitScale(model, st, v));
  }
  return 0;
}

int RunDry(const Cfg& c, Shared& s) {
  Teleop tp(c, s);
  double pos[3] = {0, 0, 0}, out[6];
  if (c.abs_box) {
    for (int i = 0; i < 3; ++i) pos[i] = 0.5 * (c.abs_lo[i] + c.abs_hi[i]);
    tp.SetAbsBox();
  } else {
    tp.SetHome(pos);
  }
  s.mon[12] = 1.0;
  std::thread mon(MonitorLoop, std::ref(s), std::ref(tp));
  std::thread grip;
  if (c.gripper) grip = std::thread(GripperLoop, std::cref(c), std::ref(s), true);
  auto next = std::chrono::steady_clock::now();
  double t = 0.0;
  const double dt = 0.001;
  while (true) {
    next += std::chrono::milliseconds(1);
    std::this_thread::sleep_until(next);
    t += dt;
    bool stopping = g_stop || s.dead || (c.max_s > 0 && t > c.max_s);
    tp.Step(dt, pos, stopping, out);
    for (int i = 0; i < 3; ++i) pos[i] += out[i] * dt;
    if (stopping && tp.AtRest()) break;
  }
  g_stop = true;
  mon.join();
  if (grip.joinable()) grip.join();
  return 0;
}

int RunRobot(const Cfg& c, Shared& s) {
  franka::Robot robot(c.ip, c.realtime ? franka::RealtimeConfig::kEnforce
                                       : franka::RealtimeConfig::kIgnore);
  if (c.recover) robot.automaticErrorRecovery();
  robot.setCollisionBehavior({{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                             {{40.0, 40.0, 36.0, 36.0, 32.0, 28.0, 24.0}},
                             {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                             {{40.0, 40.0, 40.0, 50.0, 50.0, 50.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});

  franka::Model model = robot.loadModel();
  Teleop tp(c, s);
  std::thread mon(MonitorLoop, std::ref(s), std::ref(tp));
  std::thread grip;
  if (c.gripper) grip = std::thread(GripperLoop, std::cref(c), std::ref(s), false);
  double t = 0.0;
  int rc = 0;
  try {
    robot.control(
        [&](const franka::RobotState& st, franka::Duration dur) -> franka::CartesianVelocities {
          double pos[3] = {st.O_T_EE[12], st.O_T_EE[13], st.O_T_EE[14]};
          if (!tp.have_home) {
            if (c.abs_box) tp.SetAbsBox();
            else tp.SetHome(pos);
          }
          t += dur.toSec();
          bool stopping = g_stop || s.dead || (c.max_s > 0 && t > c.max_s);
          double out[6];
          tp.Step(dur.toSec(), pos, stopping, out);
          double jl = JointLimitScale(model, st, out);
          for (double& x : out) x *= jl;
          s.mon[12] = jl;
          franka::CartesianVelocities cmd({out[0], out[1], out[2], out[3], out[4], out[5]});
          if (stopping && tp.AtRest()) return franka::MotionFinished(cmd);
          return cmd;
        },
        franka::ControllerMode::kCartesianImpedance, true, franka::kDefaultCutoffFrequency);
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka exception: %s\n", e.what());
    rc = 1;
  }
  g_stop = true;
  mon.join();
  if (grip.joinable()) grip.join();
  return rc;
}

}  // namespace

int main(int argc, char** argv) {
  Cfg c;
  if (!ParseArgs(argc, argv, c)) return 2;
  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  try {
    if (c.status) return RunStatus(c);

    std::string dev = c.device.empty() ? FindHidraw() : c.device;
    if (dev.empty()) {
      std::fprintf(stderr, "no SpaceMouse motion interface found (is it connected and awake?)\n");
      return 3;
    }
    int fd = open(dev.c_str(), O_RDONLY);
    if (fd < 0) {
      std::fprintf(stderr, "cannot open %s: %s (udev rule for vendor 256f needed?)\n", dev.c_str(),
                   std::strerror(errno));
      return 3;
    }
    std::printf("SpaceMouse on %s | mode=%s rot=%d vmax=%.3f wmax=%.3f yaw=%.0fdeg box=+-%.2f "
                "z[-%.2f,+%.2f] max=%.0fs\n",
                dev.c_str(), c.dry_run ? "DRY-RUN" : "ROBOT", (int)c.rot, c.vmax, c.wmax, c.yaw_deg,
                c.box_xy, c.box_z_down, c.box_z_up, c.max_s);
    if (c.abs_box)
      std::printf("absolute box (base frame): x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]\n", c.abs_lo[0],
                  c.abs_hi[0], c.abs_lo[1], c.abs_hi[1], c.abs_lo[2], c.abs_hi[2]);
    std::fflush(stdout);

    Shared s;
    std::thread reader(ReaderLoop, fd, std::ref(s));
    int rc = 1;
    try {
      rc = c.dry_run ? RunDry(c, s) : RunRobot(c, s);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "error: %s\n", e.what());
    }
    g_stop = true;
    reader.join();
    close(fd);
    return rc;
  } catch (const franka::Exception& e) {
    std::fprintf(stderr, "franka exception: %s\n", e.what());
    return 1;
  }
}
