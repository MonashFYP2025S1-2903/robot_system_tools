# franka_control_suite
**This repo is a work in progress **

Contains lowlevel torque controllers for Franka Emika panda and an interprocess communication interface using ZeroMQ. This code base's intended functionality is similar to [franka-interface](https://github.com/iamlab-cmu/franka-interface) but without any dependancy in ROS.


# Dependencies 

- libfranka
- ZeroMQ
- Eigen




To do:
- [ ] add python example test scripts 
- [ ] add interpolators to all implementations

Notes: 
- IK controller has been tested in the real world. 3 methods have been implemented
    - Moore-Penrose Inverse
    - Damped Least Square 
    - Jacobian transpose
- Joint torque controller listed to desired joint torque commands, clips it to real robot limits and limits the rate before applying.
- other controllers are being tested currently
- fixed impedance OSC has been implemented completely, variable impedance and K_p are partially implemented.

## Xbox controller teleop (`lingheng/franka-deployment` fork)

Confirmed working end-to-end on the Franka workstation, `men119` account, 2026-09-14.

### One-time account setup

Needed once per Linux account that will run `franka_control` (both are Linux group
memberships -- log out and back in, or reconnect via SSH, after adding either):
```bash
sudo usermod -aG realtime <account>   # 1 kHz control loop needs SCHED_FIFO priority
sudo usermod -aG input <account>      # pygame needs to see /dev/input/js*, /dev/input/event*
```
**Gotcha found 2026-09-14**: on this workstation, `ulimit -r` (the `realtime` group's rtprio
grant) only actually applies to sessions authenticated through `sshd`'s PAM stack -- a *local
desktop login* (GDM) does not pick it up even after a genuine full log-out/log-in, despite
`gdm-password`'s and `gdm-autologin`'s PAM configs both already including `pam_limits.so`
(checked directly; not a missing-config issue, root cause not fully identified). If
`franka_control` fails with `libfranka: unable to set realtime scheduling: Operation not
permitted` from a local terminal, the actual fix is a file capability, not a session/login
change (see "Rebuilding" below).

Python side: a plain `venv` is enough (`zmq`/`numpy`/`scipy`/`pygame` only) --
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install pyzmq numpy scipy pygame
```

### Building

```bash
mkdir -p build && cd build
cmake .. && make franka_control -j$(nproc)
sudo setcap cap_sys_nice+ep ./franka_control   # see gotcha above -- required every rebuild,
                                                #   since setcap's file attribute is wiped when
                                                #   the binary is replaced
```

### Running

```bash
# Terminal 1 -- the real-time controller (moves the arm to a fixed rest pose on startup)
./franka_control <robot_ip> <realtime_pc_ip> <workstation_ip> [gripper_speed gripper_force]
#   e.g. on the Franka workstation itself, controller + Python client both local:
./franka_control 172.16.0.2 127.0.0.1 127.0.0.1
#   the optional trailing args are only the cold-start default (before any client has sent a
#   command) -- once xbox_teleop.py is running, ITS --gripper-speed/--gripper-force take over
#   live (sent every tick over the same ZMQ channel as the arm pose), so day-to-day tuning only
#   needs restarting the Python script, not franka_control.

# Terminal 2 -- the Xbox teleop client
python example_python_scripts/xbox_teleop.py --max-lin-vel 1.5 --max-rot-vel 2.0 --gripper-speed 0.1 --gripper-force 20.0
```
Robot must be unlocked and FCI activated via the Desk web UI (`https://<robot_ip>/desk/`) first.
A=close gripper, B=open, Back/Select=quit. See the script's own docstring for the full ZMQ
protocol and the fixes found along the way (cold-start crash, axis mapping, state dtype).
