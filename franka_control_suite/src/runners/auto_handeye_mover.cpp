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

    // Per-joint perturbation half-ranges (rad): wrist (5,6,7) largest for rotation variation;
    // base/arm small for position diversity; q2,q3 tight for safety/visibility.
    std::array<double, 7> dq = {0.20, 0.10, 0.10, 0.15, 0.40, 0.40, 0.50};
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

        const std::array<double, 7> q0 = robot.readOnce().q;

        // Generate perturbed configs around the current pose q0
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        std::vector<std::array<double, 7>> poses;
        for (int i = 0; i < n_poses; ++i) {
            std::array<double, 7> q = q0;
            for (int j = 0; j < 7; ++j) {
                q[j] = q0[j] + u(rng) * dq[j];
                q[j] = std::max(q_min[j], std::min(q_max[j], q[j]));
            }
            poses.push_back(q);
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

            if (cmd == "done" || idx >= n_poses) {
                std::cout << "Returning to start pose...\n";
                try { MotionGenerator home(0.15, q0); robot.control(home); }
                catch (const franka::Exception& e) { std::cout << "home move: " << e.what() << "\n"; }
                const std::array<double, 16> ee = robot.readOnce().O_T_EE;
                zmq::message_t rep(ee.size() * sizeof(double));
                std::memcpy(rep.data(), ee.data(), ee.size() * sizeof(double));
                sock.send(rep);
                break;
            }

            std::cout << "Moving to pose " << idx << "/" << n_poses << std::endl;
            try {
                MotionGenerator mg(0.15, poses[idx]);
                robot.control(mg);
            } catch (const franka::Exception& e) {
                std::cout << "  move failed: " << e.what() << " — recovering\n";
                try { robot.automaticErrorRecovery(); } catch (...) {}
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(800));  // settle

            const std::array<double, 16> ee = robot.readOnce().O_T_EE;
            ++idx;
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
