// joint_pos_runner.cpp — JOINT-POSITION runner for hri-pl-frm-mvvd policy deployment.
//
// Reference copy (per docs/01_deployment_setup.md). Lives on the workstation at
//   ~/robot_system_tools/franka_control_suite/src/runners/joint_pos_runner.cpp
//
// CONTROL MODE: libfranka POSITION control (franka::JointPositions), NOT torque impedance.
//   The torque-impedance controller (JointPosition / joint_pos.cpp) jerked at startup and tripped the
//   joint_velocity_violation reflex even with the Franka-standard gains — torque control on this setup is
//   finicky. Position control lets libfranka handle the low-level torque + smoothing, so there's no gain
//   tuning and no startup jerk. We just RATE-LIMIT the commanded position (velocity + acceleration caps)
//   toward the goal so libfranka's smoothness reflexes don't trip. (Trade-off: position control is stiff,
//   not compliant — fine for a first deployment; revisit impedance later if contact compliance is needed.)
//
// ZMQ interface (matches docs/aruco/deploy_zmq.py):
//   ACTION  (SUB, connect, :2069)  JOINT_ANGLES_GRIPPER = float64[q1..q7, g_open, g_cmd]
//                                  -> deploy_zmq.py BINDs its action PUB.
//   STATE   (PUB, bind,    :2096)  float64[q(7), O_T_EE(16), grip_w(1), ext_wrench(6)] = 30
//                                  -> deploy SUBs (derives velocity by finite-diff; ext_wrench added
//                                  2026-08-10 for wrench-observation policies, see ext_wrench note below).
//
// CMake target (on the workstation, mirrors read_states; joint_pos.cpp left in the list but now unused):
//   add_executable(joint_pos_runner src/runners/joint_pos_runner.cpp src/controllers/joint_pos.cpp
//                  src/common.cpp src/communication/action_subscriber.cpp src/communication/state_publisher.cpp)
//   target_link_libraries(joint_pos_runner PUBLIC Franka::Franka libzmq ${Franka} ${cppzmq} Eigen3::Eigen)
//
// RUN (Franka workstation, FCI active/blue; deploy_zmq.py on the same host so ZMQ is localhost):
//   Terminal 1:  cd ~/robot_system_tools/franka_control_suite/build
//                ./joint_pos_runner 172.16.0.2 127.0.0.1 '*'
//   Terminal 2:  cd ~/hri-pl-frm-mvvd
//                python docs/aruco/deploy_zmq.py --policy <onnx> --calib calibration/camera_to_base.npz \
//                       --use-velocity --ctrl-ip 127.0.0.1 [--action-scale 0.25]
//
// ARGUMENTS  ./joint_pos_runner <robot_ip> <action_connect_ip> <state_bind_ip>   (ports fixed: actions :2069, state :2096)
//   1 robot_ip           Franka FCI address — libfranka connects here (e.g. 172.16.0.2).
//   2 action_connect_ip  IP the ActionSubscriber (SUB) CONNECTs to, to RECEIVE joint targets = where deploy
//                        PUBLISHES them. deploy runs on this host -> 127.0.0.1.
//   3 state_bind_ip      interface the StatePublisher (PUB) BINDs to. '*' = all interfaces (quote it).
//
// ⚠ SAFETY: TRACK_VMAX/TRACK_AMAX keep motion gentle; the goal is seeded to q_default so the arm holds at
//   default until the policy publishes. Test arm-only (USE_GRIPPER=false) first, hand on the e-stop.

#include <iostream>
#include <thread>
#include <atomic>
#include <array>
#include <vector>
#include <chrono>
#include <algorithm>
#include <cmath>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>
#include <franka/control_types.h>
#include <franka/duration.h>

#include <Eigen/Dense>   // z-only freeze (2026-08-06): Jacobian pseudo-inverse, see JointPositionTracker

#include "context.h"
#include "common.h"                              // MotionGenerator, setDefaultBehavior
#include "communication/action_subscriber.h"
#include "communication/state_publisher.h"

namespace robotContext { franka::Robot *robot; franka::Gripper *gripper; franka::Model *model; }
namespace Comms        { ActionSubscriber *actionSubscriber; StatePublisher *statePublisher; }

// IsaacLab Franka-lift default joint config (start here; MUST match Q_DEFAULT_ARM in deploy_zmq.py).
static const std::array<double, 7> Q_DEFAULT = {{0.0, -0.569, 0.0, -2.810, 0.0, 3.037, 0.741}};
// Franka Panda joint position limits (true datasheet values).
static const std::array<double, 7> JMIN_RAW = {{-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973}};
static const std::array<double, 7> JMAX_RAW = {{ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973}};
// SAFETY MARGIN (2026-08-05): repeated joint_motion_generator_position_limits_violation reflex trips even
// though q_cmd is clamped to JMIN_RAW/JMAX_RAW every iteration -- with the tracker dwelling AT a limit for many
// seconds (an erratic/wide-swinging policy approach), accumulated float error in the per-iteration
// `q_cmd += v_cmd*dt` then clamp can land epsilon outside the true limit, which our clamp reproduces exactly
// but libfranka's own (stricter, internal) check can still reject. Inset the SOFTWARE limits so no accumulated
// slop ever gets close enough to the TRUE hard limit to matter.
static constexpr double LIMIT_MARGIN = 0.05;   // rad (~3 deg)
static std::array<double, 7> JMIN, JMAX;       // = JMIN_RAW+MARGIN / JMAX_RAW-MARGIN, filled in main()

static constexpr bool   USE_GRIPPER = true;      // ⚠ keep false for the first arm-only test
static constexpr double MOVE_SPEED  = 0.1;       // MotionGenerator speed_factor [0..1] for the move to q_default
static double TRACK_VMAX  = 0.2;                 // rad/s   DEFAULT max joint speed; NOW overridable via argv[4] (SLOW + safe;
                                                 //         raise later if the arm lags the policy too much)
static constexpr double TRACK_AMAX  = 2.0;       // rad/s^2 max joint acceleration (gentle)
static constexpr double SOFTSTART_T = 1.5;
static double Z_FLOOR     = 0.065;                // m (base frame) DEFAULT; overridable via argv[5]. table-floor guard: freeze descent if the
                                                 //   fingertip world-z (O_T_EE.z + 0.1034*z_axis) drops below this.       // s — ramp tracking speed 0->full at startup (no handoff jerk)
static std::atomic<bool> g_running{true};
static std::atomic<double> g_grip_w{-1.0};   // latest gripper width (m); published in the state stream for grasp-verify
static double CONTACT_FN = 10.0;             // N — external z-force that counts as "contact"; freeze descent above it
                                             //   so the arm never pushes into the cube hard enough to trip a reflex.
                                             //   Overridable via argv[6]. 0 disables.
static double JOINT_STIFFNESS = 80.0;        // Nm/rad — sim2real match (2026-08-05): setDefaultBehavior() leaves the
                                             //   robot at libfranka's example joint impedance {3000,3000,3000,2500,
                                             //   2500,2000,2000} Nm/rad, but the policy was TRAINED in sim against
                                             //   ImplicitActuatorCfg stiffness=80 (both panda_shoulder AND
                                             //   panda_forearm, franka.py FRANKA_PANDA_CFG) -- ~30-37x softer than
                                             //   what's actually running on hardware. Matching it here so the real
                                             //   joint controller's compliance matches what the policy expects,
                                             //   instead of tuning deploy-side thresholds around the mismatch.
                                             //   Overridable via argv[7].

// Position-control functor: rate-limit commanded q toward the goal (velocity + acceleration capped) so the
// motion is smooth and libfranka's reflexes don't trip. For a static goal (q_default) it just holds.
struct JointPositionTracker {
    std::array<double, 7> q_cmd{}, v_cmd{};
    bool init = false;
    double t_run = 0.0;
    std::size_t count = 1;
    double fz_prev = 1e9;
    franka::JointPositions operator()(const franka::RobotState& rs, franka::Duration period) {
        double dt = period.toSec();
        if (dt <= 0.0) dt = 1e-3;
        if (!init) { q_cmd = rs.q; v_cmd.fill(0.0); init = true; }   // start at the actual robot q (continuity)
        t_run += dt;
        double vmax = TRACK_VMAX * std::min(1.0, t_run / SOFTSTART_T);  // soft-start: ease speed in -> no handoff jerk

        // table-floor guard: fingertip world-z = O_T_EE translation_z + TCP(0.1034 m) * z_axis_z (column-major).
        double fz = rs.O_T_EE[14];   // O_T_EE IS the fingertip TCP already (verified by touch-test) -> no +0.1034
        double fext_z = rs.O_F_ext_hat_K[2];   // external z-force on the EE (base frame), N; >0 => pressing DOWN on an object
        bool contact = (CONTACT_FN > 0.0) && (fext_z > CONTACT_FN);     // fingertip hit the cube/table
        bool block_descent = ((fz < Z_FLOOR) && (fz <= fz_prev + 1e-4)) // table-floor guard (height)
                             || contact;                                // contact guard: stop pushing -> no reflex lock
        fz_prev = fz;
        if (block_descent && (count % 100 == 0))
            std::cout << (contact ? "[contact] fext_z=" : "[Z-floor] fingertip z=")
                      << (contact ? fext_z : fz) << (contact ? "N -> blocking further DESCENT (z-only, avoid reflex)"
                                                             : " < Z_FLOOR -> blocking further DESCENT (z-only)") << std::endl;

        std::vector<double> goal;
        Comms::actionSubscriber->readValues(goal);                  // 9 = 7 q + 2 grip; first 7 are the joints
        bool goal_ok = goal.size() >= 7;                             // malformed/short read -> hold q_cmd, don't touch garbage
        for (int i = 0; i < 7; ++i) {
            double g  = goal_ok ? std::clamp(goal[i], JMIN[i], JMAX[i]) : q_cmd[i];  // goal clamped to (margined) limits
            double vd = std::clamp((g - q_cmd[i]) / dt, -vmax, vmax);                // velocity toward goal, capped
            v_cmd[i] += std::clamp(vd - v_cmd[i], -TRACK_AMAX * dt, TRACK_AMAX * dt); // acceleration-limited
        }

        // Z-ONLY freeze (2026-08-06, replaces the old all-7-joints freeze). The old code zeroed EVERY joint's
        // motion the instant block_descent triggered, so the arm couldn't even shuffle sideways to close the
        // last few cm to the cube once pinned at Z_FLOOR -- confirmed stuck at |gap|~8-9cm, never latching,
        // across every real-robot test regardless of stiffness. Instead: project the Cartesian velocity implied
        // by v_cmd through the Jacobian, zero ONLY the downward-z component when block_descent is active, then
        // map back to joint space via the damped pseudo-inverse -- descent stays blocked (the actual safety
        // property we want) while lateral/upward correction remains free. Also incidentally fixes the earlier
        // "v_cmd keeps accumulating during the freeze, lurches when released" bug, since v_cmd itself is now
        // corrected here rather than left to build up unconstrained behind a boolean gate.
        if (block_descent) {
            std::array<double, 42> J_flat = robotContext::model->zeroJacobian(franka::Frame::kEndEffector, rs);
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> J(J_flat.data());   // column-major, libfranka convention
            Eigen::Matrix<double, 7, 1> v_eig;
            for (int i = 0; i < 7; ++i) v_eig(i) = v_cmd[i];
            Eigen::Matrix<double, 6, 1> v_cart = J * v_eig;               // [vx,vy,vz,wx,wy,wz] implied by v_cmd
            if (v_cart(2) < 0.0) {                                        // implies further descent -> block just this
                // SMOOTH BLEND (2026-08-06): a hard on/off toggle here caused visible jitter on hardware --
                // any noise in v_cmd near the v_cart.z==0 boundary flips between "raw v_cmd" and "corrected
                // v_cmd" every 1kHz cycle, and those two can differ meaningfully even for similar inputs.
                // Ramp the correction strength smoothly over a small band instead of snapping to full
                // correction the instant it crosses zero (blend==0 exactly at v_cart.z>=0, matching the old
                // gate's boundary -- only the APPROACH to full correction is now continuous).
                constexpr double BLEND_MARGIN = 0.02;  // rad/s -- width of the smooth transition band
                double blend = std::clamp(-v_cart(2) / BLEND_MARGIN, 0.0, 1.0);
                Eigen::Matrix<double, 6, 1> v_cart_corrected = v_cart;
                v_cart_corrected(2) = 0.0;
                Eigen::Matrix<double, 6, 6> damped =
                    J * J.transpose() + 1e-6 * Eigen::Matrix<double, 6, 6>::Identity();  // damped for near-singular safety
                Eigen::Matrix<double, 7, 1> v_corrected = J.transpose() * damped.inverse() * v_cart_corrected;  // min-norm
                for (int i = 0; i < 7; ++i) v_cmd[i] = (1.0 - blend) * v_cmd[i] + blend * v_corrected(i);
            }
        }

        bool near_limit = false;
        for (int i = 0; i < 7; ++i) {
            q_cmd[i] += v_cmd[i] * dt;
            q_cmd[i]  = std::clamp(q_cmd[i], JMIN[i], JMAX[i]);
            if (q_cmd[i] < JMIN[i] + 0.01 || q_cmd[i] > JMAX[i] - 0.01) near_limit = true;  // within 0.01 rad of our margined bound
        }
        if (near_limit && (count % 100 == 0))
            std::cout << "[near-limit] q_cmd=[" << q_cmd[0]<<","<<q_cmd[1]<<","<<q_cmd[2]<<","<<q_cmd[3]<<","
                      << q_cmd[4]<<","<<q_cmd[5]<<","<<q_cmd[6] << "]  JMIN/JMAX(margined)" << std::endl;
        if ((count++ % 4) == 0) {                                   // publish q (7) at ~250 Hz
            std::vector<double> msg(rs.q.begin(), rs.q.end());
            msg.insert(msg.end(), rs.O_T_EE.begin(), rs.O_T_EE.end());  // + O_T_EE(16): EE pose for TCP diagnostic
            msg.push_back(g_grip_w.load());                             // + gripper width(1): for closed-loop grasp-verify
            // + ext_wrench(6) [Fx,Fy,Fz,Tx,Ty,Tz] (2026-08-10): libfranka's O_F_ext_hat_K, the estimated
            // EXTERNAL wrench at the stiffness/EE frame, expressed in the BASE frame O (already read above
            // for the CONTACT_FN check -- only fext_z was used there, this publishes the full 6D vector).
            // CAVEAT, not yet empirically validated: the wrench-observation policies were trained against
            // PhysX's body_incoming_joint_wrench_b at panda_hand -- the wrench transmitted through that
            // body's incoming joint, expressed in the BODY's OWN frame. That is NOT the same physical
            // quantity as libfranka's external-wrench estimate (different frame: body vs base; different
            // estimation method: internal joint-constraint force vs dynamics-residual-based external-force
            // estimate). This is the closest available real-hardware proxy without added F/T sensor
            // hardware, but treat sign/scale/frame as unverified until checked against known contact events.
            msg.insert(msg.end(), rs.O_F_ext_hat_K.begin(), rs.O_F_ext_hat_K.end());
            Comms::statePublisher->writeMessage(msg);
        }
        return franka::JointPositions(q_cmd);
    }
};

int main(int argc, char* argv[]) {
    if (argc < 4 || argc > 8) {
        std::cout << "usage: ./joint_pos_runner <robot_ip> <action_connect_ip> <state_bind_ip> [track_vmax] [z_floor] [contact_N] [joint_stiffness]\n"
                     "  e.g.  ./joint_pos_runner 172.16.0.2 127.0.0.1 '*' 0.30 0.035 10 80   (vmax def 0.2; z_floor def 0.065; contact_N def 10, 0=off; "
                     "joint_stiffness def 80 Nm/rad = sim match, libfranka factory default is ~2000-3000)\n";
        return 1;
    }
    const std::string robot_ip = argv[1], action_ip = argv[2], state_ip = argv[3];
    if (argc >= 5) TRACK_VMAX      = std::stod(argv[4]);   // sweep vmax live, no rebuild
    if (argc >= 6) Z_FLOOR         = std::stod(argv[5]);   // sweep grasp floor live, no rebuild
    if (argc >= 7) CONTACT_FN      = std::stod(argv[6]);   // sweep contact-force stop live, no rebuild
    if (argc >= 8) JOINT_STIFFNESS = std::stod(argv[7]);   // sweep joint impedance live, no rebuild
    for (int i = 0; i < 7; ++i) { JMIN[i] = JMIN_RAW[i] + LIMIT_MARGIN; JMAX[i] = JMAX_RAW[i] - LIMIT_MARGIN; }
    std::cout << "[cfg] TRACK_VMAX = " << TRACK_VMAX << " rad/s   Z_FLOOR = " << Z_FLOOR
              << " m   CONTACT_FN = " << CONTACT_FN << " N   LIMIT_MARGIN = " << LIMIT_MARGIN
              << " rad   JOINT_STIFFNESS = " << JOINT_STIFFNESS << " Nm/rad" << std::endl;

    try {
        ActionSubscriber as_(CommsDataType::JOINT_ANGLES_GRIPPER, std::string("tcp://") + action_ip + ":2069");
        StatePublisher   sp_(std::string("tcp://") + state_ip + ":2096");
        // SAFETY: seed the goal with q_default (+ gripper open) so the arm holds at default until the policy publishes.
        as_.values = {Q_DEFAULT[0], Q_DEFAULT[1], Q_DEFAULT[2], Q_DEFAULT[3],
                      Q_DEFAULT[4], Q_DEFAULT[5], Q_DEFAULT[6], 0.04, 0.04};
        Comms::actionSubscriber = &as_;
        Comms::statePublisher   = &sp_;

        franka::Robot   robot_(robot_ip);
        franka::Gripper gripper_(robot_ip);
        setDefaultBehavior(robot_);
        // Raise collision thresholds above libfranka defaults (~20 N) so a firm cube/table grasp
        // contact does not trip cartesian_reflex and abort the runner before the gripper closes.
        // Still protective (slow motion + hand on e-stop). Order: joint torque (lower/upper), then
        // cartesian force (lower/upper).
        robot_.setCollisionBehavior(
            {{80,80,78,78,76,74,72}}, {{80,80,78,78,76,74,72}},
            {{90,90,90,90,90,90}},    {{90,90,90,90,90,90}});
        // Sim2real joint-impedance match (see JOINT_STIFFNESS comment) -- overrides setDefaultBehavior()'s
        // libfranka-example stiffness {3000,3000,3000,2500,2500,2000,2000} with the sim-trained value.
        robot_.setJointImpedance({{JOINT_STIFFNESS, JOINT_STIFFNESS, JOINT_STIFFNESS, JOINT_STIFFNESS,
                                   JOINT_STIFFNESS, JOINT_STIFFNESS, JOINT_STIFFNESS}});

        robotContext::robot = &robot_; robotContext::gripper = &gripper_;
        franka::Model model_ = robot_.loadModel();
        robotContext::model = &model_;

        // 1) move to the policy default config (slow), then hand off to the position tracker (same control mode).
        std::cout << "moving to q_default..." << std::endl;
        MotionGenerator mg(MOVE_SPEED, Q_DEFAULT);
        robot_.control(mg);
        std::cout << "at q_default. starting joint POSITION control. (USE_GRIPPER=" << USE_GRIPPER << ")" << std::endl;

        // 2) action-receive thread. readMessage() blocks on recv(); wrap in try/catch so the ZMQ-context
        //    teardown on shutdown exits the (detached) thread quietly instead of std::terminate.
        std::thread subThread([]() {
            try { while (g_running.load()) Comms::actionSubscriber->readMessage(); }
            catch (...) {}
        });
        subThread.detach();

        // 3) gripper thread — OPTIONAL, off by default (see USE_GRIPPER).
        if (USE_GRIPPER) {
            std::thread gripThread([&gripper_]() {
                double max_w = gripper_.readOnce().max_width;
                bool closed = true;   // assume CLOSED at startup so the FIRST open cmd actually opens (the
                                      //   physical gripper is usually left closed from the previous grasp);
                                      //   starting false made the open branch never fire -> gripper stayed shut.
                while (g_running.load()) {
                    g_grip_w.store(gripper_.readOnce().width);   // publish current width for closed-loop grasp-verify
                    double g = Comms::actionSubscriber->readGripperCommands();   // TODO: verify open/close convention
                    try {
                        if (g < 0.0 && !closed)      { gripper_.grasp(0.0, 0.1, 20.0, 0.05, 0.05); closed = true; }
                        else if (g >= 0.0 && closed) { gripper_.move(max_w, 0.1);                   closed = false; }
                    } catch (const franka::Exception&) {}
                    std::this_thread::sleep_for(std::chrono::milliseconds(150));
                }
            });
            gripThread.detach();
        }

        // 4) position-control loop — rate-limited tracking of the goal; runs until a reflex/error throws.
        std::cout << "goal seed (expect q_default): ";
        for (int i = 0; i < 7; ++i) std::cout << as_.values[i] << (i < 6 ? ", " : "\n");
        JointPositionTracker tracker;
        robot_.control(tracker);

        g_running.store(false);

    } catch (const franka::Exception& e) {
        g_running.store(false);
        std::cout << "franka exception: " << e.what() << std::endl;
        return -1;
    }
    return 0;
}
