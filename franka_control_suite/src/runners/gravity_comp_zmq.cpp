/**
 * gravity_comp_zmq.cpp
 *
 * Runs gravity compensation on the Franka (enabling hand-guiding) while
 * serving the current EE pose over a ZMQ REP socket in a background thread.
 *
 * Build: see CMakeLists.txt (target: gravity_comp_zmq)
 * Usage: ./gravity_comp_zmq <robot_ip> <zmq_port>
 * Example: ./gravity_comp_zmq 172.16.0.2 2000
 *
 * With this running:
 *   - Hold the enabling button → robot is compliant, freely movable
 *   - Python calibration script sends ZMQ "data" request → gets O_T_EE
 *   - No FCI toggling needed between poses
 *
 * Stop: Ctrl+C (robot stiffens immediately)
 */

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/rate_limiting.h>

#include <zmq.hpp>

#include <array>
#include <atomic>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

static std::array<double, 16> g_ee_pose{};
static std::mutex              g_pose_mutex;
static std::atomic<bool>       g_running{true};

void pose_server_thread(const std::string& port) {
    zmq::context_t ctx(1);
    zmq::socket_t  sock(ctx, ZMQ_REP);
    sock.bind("tcp://*:" + port);
    std::cout << "ZMQ state server bound on port " << port << std::endl;

    while (g_running) {
        zmq::pollitem_t items[] = {{static_cast<void*>(sock), 0, ZMQ_POLLIN, 0}};
        zmq::poll(items, 1, 100);   // 100 ms timeout — allows g_running check

        if (!(items[0].revents & ZMQ_POLLIN)) continue;

        zmq::message_t request;
        sock.recv(&request);

        std::array<double, 16> pose;
        {
            std::lock_guard<std::mutex> lock(g_pose_mutex);
            pose = g_ee_pose;
        }

        zmq::message_t reply(pose.size() * sizeof(double));
        std::memcpy(reply.data(), pose.data(), pose.size() * sizeof(double));
        sock.send(reply);
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot_ip> <zmq_port>\n";
        return 1;
    }
    const std::string robot_ip  = argv[1];
    const std::string zmq_port  = argv[2];

    std::thread zmq_t(pose_server_thread, zmq_port);

    try {
        franka::Robot robot(robot_ip);
        franka::Model model(robot.loadModel());

        std::cout << "Gravity compensation running.\n"
                  << "Hold enabling button to hand-guide freely.\n"
                  << "Ctrl+C to stop.\n";

        robot.control([&](const franka::RobotState& state,
                          franka::Duration) -> franka::Torques {
            {
                std::lock_guard<std::mutex> lock(g_pose_mutex);
                g_ee_pose = state.O_T_EE;
            }
            auto grav = model.gravity(state);
            const std::array<double, 7> damping = {5.0, 5.0, 5.0, 5.0, 3.0, 2.0, 1.0};
            for (size_t i = 0; i < 7; i++) { grav[i] -= damping[i] * state.dq[i]; }
            return franka::Torques(grav);
        });

    } catch (const franka::Exception& e) {
        std::cerr << e.what() << std::endl;
        g_running = false;
        zmq_t.join();
        return -1;
    }

    g_running = false;
    zmq_t.join();
    return 0;
}
