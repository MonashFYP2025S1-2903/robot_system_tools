#include <iostream>

#include "context.h"
#include "controllers/ik.h"
#include <franka/exception.h>
#include "common.h"
#include <thread>

namespace robotContext {
    franka::Robot *robot;
    franka::Gripper *gripper;
    franka::Model *model;

}
namespace Comms {
    ActionSubscriber *actionSubscriber; 
    StatePublisher *statePublisher; 
}

int main(int argc, char* argv[]) {

    std::string robot_ip, realtime_pc_ip, workstation_ip; 

    if(argc != 4){
        std::cout<<"The usage is ./frank_control robot_ip realtime_pc_ip workstation_ip \n";
        exit(1);
    }
    else{
        robot_ip = argv[1]; 
        realtime_pc_ip = argv[2]; 
        workstation_ip = argv[3];
    }
    try {
        ActionSubscriber as_(CommsDataType::POSE, std::string("tcp://") + realtime_pc_ip + std::string(":2069"));
        StatePublisher sp_(std::string("tcp://") + workstation_ip + std::string(":2096"));
        Comms::actionSubscriber = &as_; 
        Comms::statePublisher = &sp_; 
        franka::Robot robot_(robot_ip);
        franka::Gripper gripper_(robot_ip);

        robotContext::robot = &robot_; 
        robotContext::gripper = &gripper_; 
        franka::Model model_ = robotContext::robot->loadModel();
        robotContext::model = &model_;         
        std::cout << "moving robot to default position..." << std::endl;
        std::array<double, 7> qRest = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motionGenerator(0.1, qRest);
        robotContext::robot->control(motionGenerator);
        std::cout << "finished moving robot to default position" << std::endl;

        // Prime the command buffer with the robot's actual current pose before starting the
        // control loop. ActionSubscriber defaults to all-zeros, which under the default
        // ControlMode::ABSOLUTE (see base_controller.h) means "target = world origin" for
        // position and an invalid zero-norm quaternion for orientation -- neither is a safe
        // "no command received yet" value. Found 2026-09-14: the zero-quaternion normalize()
        // threw a non-franka::Exception on the very first control tick (no client had connected
        // yet), which propagated uncaught past the catch(franka::Exception&) below and unwound
        // through the never-joined subscribeThread, calling std::terminate() ("terminate called
        // without an active exception"). Priming with the real current pose means holding still
        // is the correct behavior even before any teleop client has sent a single command.
        franka::RobotState initial_state = robotContext::robot->readOnce();
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        Eigen::Quaterniond initial_quat(initial_transform.linear());
        initial_quat.normalize();
        Comms::actionSubscriber->values = {
            initial_transform.translation().x(), initial_transform.translation().y(), initial_transform.translation().z(),
            initial_quat.x(), initial_quat.y(), initial_quat.z(), initial_quat.w()
        };

        InverseKinematics IK_(1, IKType::M_P_PSEUDO_INVERSE);
        std::thread subscribeThread([]() {
            while(true) {
                Comms::actionSubscriber->readMessage();
            }
        });        
        while(true) {
            robotContext::robot->control(IK_);
        }
    
    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }
}

