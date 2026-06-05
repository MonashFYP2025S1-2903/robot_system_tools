/**
 * auto_handeye_mover.cpp
 *
 * Automatic hand-eye calibration mover. Drives the Franka through N joint configurations that
 * are SMALL perturbations around the robot's CURRENT pose (mostly wrist joints, for the rotation
 * variation hand-eye needs), and serves the EE pose over a ZMQ REP socket so a Python client can
 * capture the camera marker at each settled pose. The Python side does detection + calibrateHandEye.
 *
 * ── SAFETY ───────────────────────────────────────────────────────────────────────────────────
 *   - Hand-guide the robot to a SAFE start first: EE marker facing the front camera, arm roughly
 *     centred, clear of obstacles. The perturbations are centred on THIS pose.
 *   - Motion is slow (speed factor 0.15). Conservative collision behaviour stops the robot on
 *     contact. KEEP A HAND ON THE E-STOP, especially on the first run.
 *   - TEST with a small N first (e.g. 3) and watch before running the full set.
 *
 * Build: add CMake target (see CMakeLists.txt). Needs FCI active (blue light).
 * Usage:  ./auto_handeye_mover <robot_ip> <zmq_port> <n_poses> [seed] [max_dq_scale]
 * Protocol (ZMQ REP on tcp://*:<port>):
 *   client sends "next" -> mover moves to the next pose, settles, replies 16 float64 O_T_EE (col-major)
 *   client sends "done" -> mover returns to the start pose and exits (still replies once)
 */

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>

#include <zmq.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "common.h"  // MotionGenerator

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <robot_ip> <zmq_port> <n_poses> [seed] [max_dq_scale]\n";
        return 1;
    }
    const std::string robot_ip = argv[1];
    const std::string zmq_port = argv[2];
    const int   n_poses = std::stoi(argv[3]);
    const unsigned seed = (argc > 4) ? std::stoul(argv[4]) : 42u;
    const double scale  = (argc > 5) ? std::stod(argv[5]) : 1.0;  // shrink/grow perturbations

    // Per-joint perturbation half-ranges (rad). Larger ARM ranges (1,2,4) give the EE the
    // TRANSLATION spread hand-eye needs; wrist (5,6,7) gives rotation spread. The FK collision
    // checks + marker-visibility filter reject unsafe/out-of-view poses, so we can be generous.
    std::array<double, 7> dq = {0.50, 0.35, 0.25, 0.35, 0.60, 0.60, 0.70};
    for (auto& d : dq) d *= scale;

    // Franka Panda joint limits (rad) with a small margin
    const std::array<double, 7> q_min = {-2.85, -1.72, -2.85, -3.02, -2.85, 0.00, -2.85};
    const std::array<double, 7> q_max = { 2.85,  1.72,  2.85, -0.10,  2.85, 3.65,  2.85};

    try {
        franka::Robot robot(robot_ip);
        // Conservative collision thresholds -> contact stops the robot.
        robot.setCollisionBehavior(
            {{20, 20, 18, 18, 16, 16, 14}}, {{20, 20, 18, 18, 16, 16, 14}},
            {{20, 20, 18, 18, 16, 16, 14}}, {{20, 20, 18, 18, 16, 16, 14}},
            {{20, 20, 18, 18, 18, 18}},     {{20, 20, 18, 18, 18, 18}},
            {{20, 20, 18, 18, 18, 18}},     {{20, 20, 18, 18, 18, 18}});

        // --- Verify the robot is FCI-controllable (not hand-guiding / reflex / user-stopped) ---
        auto mode_name = [](franka::RobotMode m) -> std::string {
            switch (m) {
                case franka::RobotMode::kOther: return "Other";
                case franka::RobotMode::kIdle: return "Idle (FCI-ready)";
                case franka::RobotMode::kMove: return "Move";
                case franka::RobotMode::kGuiding: return "Guiding (hand-guiding)";
                case franka::RobotMode::kReflex: return "Reflex (safety stop)";
                case franka::RobotMode::kUserStopped: return "UserStopped";
                case franka::RobotMode::kAutomaticErrorRecovery: return "AutomaticErrorRecovery";
                default: return "Unknown";
            }
        };
        franka::RobotState s0 = robot.readOnce();
        int waited = 0;
        while (s0.robot_mode != franka::RobotMode::kIdle) {
            if (s0.robot_mode == franka::RobotMode::kReflex ||
                s0.robot_mode == franka::RobotMode::kUserStopped) {
                std::cerr << "Robot is " << mode_name(s0.robot_mode)
                          << " — recover in Franka Desk (dismiss errors / unlock joints), then re-run.\n";
                return 2;
            }
            if (s0.robot_mode == franka::RobotMode::kGuiding)
                std::cout << "Robot in HAND-GUIDING mode — release the enabling/guiding button to continue...\n";
            else
                std::cout << "Waiting for FCI-ready (kIdle); current mode: " << mode_name(s0.robot_mode) << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (++waited > 60) { std::cerr << "Timed out waiting for kIdle (60 s). Aborting.\n"; return 2; }
            s0 = robot.readOnce();
        }
        std::cout << "Robot is FCI-ready (kIdle). Proceeding.\n";

        const std::array<double, 7> q0 = s0.q;
        const franka::Model model = robot.loadModel();

        // Proactive collision checks via forward kinematics (no planner).
        const double z_floor    = 0.08;  // min height (m) for EE/wrist/flange above the table (base z=0)
        const double self_clear = 0.15;  // min distance (m) of EE/flange to shoulder/upper-arm joints

        auto link_xyz = [&](franka::Frame fr, const std::array<double, 7>& q) {
            const std::array<double, 16> p = model.pose(fr, q, s0.F_T_EE, s0.EE_T_K);
            return std::array<double, 3>{p[12], p[13], p[14]};  // column-major translation
        };
        auto dist = [](const std::array<double, 3>& a, const std::array<double, 3>& b) {
            return std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
        };
        auto pose_safe = [&](const std::array<double, 7>& q) -> bool {
            // table: key frames must stay above the floor
            for (franka::Frame fr : {franka::Frame::kEndEffector, franka::Frame::kFlange,
                                     franka::Frame::kJoint7, franka::Frame::kJoint6, franka::Frame::kJoint5})
                if (link_xyz(fr, q)[2] < z_floor) return false;
            // crude self-collision: EE/flange not too close to shoulder/upper-arm joints
            const auto ee = link_xyz(franka::Frame::kEndEffector, q);
            const auto fl = link_xyz(franka::Frame::kFlange, q);
            for (franka::Frame fr : {franka::Frame::kJoint1, franka::Frame::kJoint2, franka::Frame::kJoint3}) {
                const auto p = link_xyz(fr, q);
                if (dist(ee, p) < self_clear || dist(fl, p) < self_clear) return false;
            }
            return true;
        };

        if (!pose_safe(q0)) {
            std::cerr << "ABORT: start pose fails the table/self-collision check. "
                         "Hand-guide to a safer, higher start pose with clearance.\n";
            return 2;
        }

        // Generate ONLY collision-checked perturbed configs around q0
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        std::vector<std::array<double, 7>> poses;
        int attempts = 0;
        const int max_attempts = n_poses * 60;
        while (static_cast<int>(poses.size()) < n_poses && attempts < max_attempts) {
            ++attempts;
            std::array<double, 7> q = q0;
            for (int j = 0; j < 7; ++j) {
                q[j] = q0[j] + u(rng) * dq[j];
                q[j] = std::max(q_min[j], std::min(q_max[j], q[j]));
            }
            if (pose_safe(q)) poses.push_back(q);
        }
        std::cout << "Generated " << poses.size() << "/" << n_poses
                  << " collision-checked poses (" << attempts << " attempts).\n";
        if (poses.empty()) {
            std::cerr << "No safe poses found — shrink dq (5th arg <1) or pick a more open start. Aborting.\n";
            return 2;
        }

        zmq::context_t ctx(1);
        zmq::socket_t  sock(ctx, ZMQ_REP);
        sock.bind("tcp://*:" + zmq_port);
        std::cout << "auto_handeye_mover ready on port " << zmq_port << " — " << n_poses
                  << " poses around current q0 (dq scale " << scale << "). Waiting for 'next'.\n";

        int idx = 0;
        while (true) {
            zmq::message_t req;
            sock.recv(&req);
            const std::string cmd(static_cast<char*>(req.data()), req.size());

            if (cmd == "done" || idx >= static_cast<int>(poses.size())) {
                std::cout << "Returning to start pose...\n";
                try { MotionGenerator home(0.15, q0); robot.control(home); }
                catch (const franka::Exception& e) { std::cout << "home move: " << e.what() << "\n"; }
                const std::array<double, 16> ee = robot.readOnce().O_T_EE;
                zmq::message_t rep(ee.size() * sizeof(double));
                std::memcpy(rep.data(), ee.data(), ee.size() * sizeof(double));
                sock.send(rep);
                break;
            }

            // Advance to the next pose whose straight joint-space PATH is also collision-checked.
            std::array<double, 16> ee{};
            bool moved = false;
            while (idx < static_cast<int>(poses.size())) {
                const std::array<double, 7> q_cur = robot.readOnce().q;
                bool path_ok = true;
                for (int k = 1; k <= 10 && path_ok; ++k) {
                    const double a = k / 10.0;
                    std::array<double, 7> qi;
                    for (int j = 0; j < 7; ++j) qi[j] = (1.0 - a) * q_cur[j] + a * poses[idx][j];
                    if (!pose_safe(qi)) path_ok = false;
                }
                if (!path_ok) {
                    std::cout << "  pose " << idx << " path unsafe (table/self) — skipping\n";
                    ++idx;
                    continue;
                }
                std::cout << "Moving to pose " << idx << "/" << poses.size() << std::endl;
                try {
                    MotionGenerator mg(0.15, poses[idx]);
                    robot.control(mg);
                } catch (const franka::Exception& e) {
                    std::cout << "  move failed: " << e.what() << " — recovering\n";
                    try { robot.automaticErrorRecovery(); } catch (...) {}
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(800));  // settle
                ee = robot.readOnce().O_T_EE;
                ++idx;
                moved = true;
                break;
            }
            if (!moved) {  // all remaining poses skipped — send zero sentinel; client stops
                std::array<double, 16> zero{};
                zmq::message_t rep(zero.size() * sizeof(double));
                std::memcpy(rep.data(), zero.data(), zero.size() * sizeof(double));
                sock.send(rep);
                continue;
            }
            zmq::message_t rep(ee.size() * sizeof(double));
            std::memcpy(rep.data(), ee.data(), ee.size() * sizeof(double));
            sock.send(rep);
        }
        std::cout << "Done. Robot at start pose.\n";
    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    return 0;
}
