# teleop_xbox_controller

`xbox_teleop.py`: Xbox controller teleoperation of the Franka Panda. Sticks and triggers drive the end effector, A closes and B opens the gripper. It sends poses to the real-time `franka_control` controller over ZMQ.

Account setup, building `franka_control` and the run commands are in the suite README, section "Xbox controller teleop": [../README.md](../README.md). Run the script from the `franka_control_suite` folder:

```bash
python teleop_xbox_controller/xbox_teleop.py --max-lin-vel 1.5 --max-rot-vel 2.0 --gripper-speed 0.1 --gripper-force 20.0
```

The SpaceMouse equivalent, which controls the arm directly through libfranka and has its own manual, is in [../teleop_spacemouse/](../teleop_spacemouse/).
