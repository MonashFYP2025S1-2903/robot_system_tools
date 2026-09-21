# spacemouse_teleop

Direct libfranka teleoperation of the Franka Panda with a 3Dconnexion SpaceMouse Wireless: translation, yaw, tilt (pitch and roll) and a hold-to-move gripper, with a workspace box and a joint-limit guard. It is independent of `franka_control` and `xbox_teleop.py` (it does not use their ZMQ interface).

**How to use it: [MANUAL.md](MANUAL.md)** (controls with a mapping diagram, quick start, safety, troubleshooting).

Build (Franka workstation, libfranka 0.9.2 and Eigen3 installed):

```bash
mkdir -p build && cd build
cmake .. && make
./spacemouse_cartesian_teleop --status     # read-only check, no motion
```

## Related Obsidian vault notes

Plain paths, not wikilinks (this repo is outside the vault):

- `Work_Res_Sensor/CSIRO_Local_SpaceMouse_Wireless/SpaceMouse_Wireless.md` (personal note, full troubleshooting log and design decisions)
- `Work_Res_Sensor/CSIRO_Local_SpaceMouse_Wireless/SpaceMouse_Franka_Teleop_Manual.md` (Obsidian copy of MANUAL.md)
- `Work_Res_Sensor/CSIRO_Local_SpaceMouse_Wireless/SpaceMouse_Wireless_Confluence.md` (shareable Confluence version)
- `Work_Res_Robot/Franka_Panda/Franka_Workbench.md` (bench geometry)
- `Work_CSIRO/P_2026-08-28_TactileGap_PreferenceLearning/2026-09-11_OMX_Teleop_Franka_CPS_Code.md` (Xbox teleop through `franka_control`, sections 14 to 16)
