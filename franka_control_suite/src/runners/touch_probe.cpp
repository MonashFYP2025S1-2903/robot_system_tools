// touch_probe.cpp — zero-torque FLOAT mode for the hand-guide cube-touch calibration check.
//
// Makes the Franka hand-movable (gravity-compensated) and prints the live TCP = O_T_EE translation +
// 0.1034 m along the gripper z-axis (the fingertip grasp point, matching the sim ee_frame). Push the
// fingertips so they straddle/touch the cube centre, hold still, and read TCP: that is the TRUE cube
// position in the robot base frame, to compare against the camera's cube= reading.
//
// RUN (stop the runner/deploy first so the FCI is free; FCI active/blue; hand on the e-stop):
//   ./touch_probe 172.16.0.2
#include <franka/robot.h>
#include <franka/exception.h>
#include <array>
#include <cstdio>
#include <iostream>

int main(int argc, char** argv) {
    if (argc < 2) { std::cout << "usage: ./touch_probe <robot_ip>\n"; return 1; }
    try {
        franka::Robot robot(argv[1]);
        // High collision thresholds so hand-pushing the arm doesn't trip a reflex.
        robot.setCollisionBehavior(
            {{100, 100, 100, 100, 100, 100, 100}}, {{100, 100, 100, 100, 100, 100, 100}},  // joint torque (7)
            {{100, 100, 100, 100, 100, 100}},      {{100, 100, 100, 100, 100, 100}});       // cartesian force (6)
        std::cout << "FLOAT mode (zero torque). Push the gripper so the FINGERTIPS straddle/touch the cube\n"
                     "centre, hold still, and read TCP -- that is the TRUE cube position. Ctrl+C when done.\n";
        std::size_t c = 0;
        robot.control([&c](const franka::RobotState& rs, franka::Duration) -> franka::Torques {
            if (c++ % 200 == 0) {                                  // ~5 Hz print
                double tx = rs.O_T_EE[12] + 0.1034 * rs.O_T_EE[8];  // column-major: trans + 0.1034*z_axis
                double ty = rs.O_T_EE[13] + 0.1034 * rs.O_T_EE[9];
                double tz = rs.O_T_EE[14] + 0.1034 * rs.O_T_EE[10];
                std::printf("TCP=[%.3f, %.3f, %.3f]   EE_pos=[%.3f, %.3f, %.3f]\n",
                            tx, ty, tz, rs.O_T_EE[12], rs.O_T_EE[13], rs.O_T_EE[14]);
                std::fflush(stdout);
            }
            return franka::Torques(std::array<double, 7>{{0, 0, 0, 0, 0, 0, 0}});
        });
    } catch (const franka::Exception& e) {
        std::cout << "franka exception: " << e.what() << std::endl; return -1;
    }
    return 0;
}
