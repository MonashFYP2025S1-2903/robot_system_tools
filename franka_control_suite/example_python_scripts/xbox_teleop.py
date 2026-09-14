"""
Xbox controller -> Franka EE-delta teleop, via franka_control_suite's ZMQ Cartesian-impedance
interface (NOT FrankaPy — see the vault note below for why that path is explicitly avoided here).

Protocol -- confirmed 2026-09-14 by reading the actual runner (src/runners/runner.cpp, the
`franka_control` executable), not just the example_python_scripts/example_CartesianMotionControl.py
reference script (whose float32 state dtype turned out to be wrong for this runner -- see below):
    Command: ZMQ PUB  tcp://127.0.0.1:2069 -> 7x float64 [x,y,z,qx,qy,qz,qw], an ABSOLUTE target
             pose. The C++ side runs InverseKinematics(M_P_PSEUDO_INVERSE) -- genuine differential
             IK (Jacobian pseudo-inverse -> joint *velocity*), NOT Cartesian impedance control.
             A CartesianImpedance controller class exists in the source tree but no executable is
             wired to it (checked CMakeLists.txt add_executable list). Continuously publishing an
             updated absolute target *is* the streaming teleop interface either way.
    State:   ZMQ SUB  tcp://127.0.0.1:2096, CONFLATE=True -> 16x float64 (NOT float32 -- confirmed
             against StatePublisher::writeMessage's std::vector<double> -- a float32 read gets
             "cannot reshape array of size 32 into shape (4,4)", found on the first real test run),
             the current EE pose as a column-major 4x4 homogeneous transform.

Only requires zmq + numpy + scipy + pygame -- no frankapy, no autolab_core, no ROS `tf`.

Related Obsidian vault note:
  Work_CSIRO/P_2026-08-28_TactileGap_PreferenceLearning/2026-09-11_OMX_Teleop_Franka_CPS_Code.md
  Work_CSIRO/P_MonashUni_FYP2025S1_2024-10-20/Allocated_FYP2025S1-2903_PL_Multiview_Data/
    FYP2025S1-2903_codebase_analysis/FYP2025S1-2903_sim2real_deployment_2026-05-18.md
  -- the latter is *why* this uses franka_control_suite's ZMQ interface and not FrankaPy: that
  note explicitly says "Ignore: robot_frankapy.py and all FrankaPy-dependent code" because
  FrankaPy needs its own Docker environment that may not be set up, whereas franka_control_suite
  is the already-vetted, ROS-free, Docker-free path used for this project's real deployment.

Run ON THE SAME MACHINE as the running franka_control_suite controller process (ZMQ PUB/SUB bind
to 127.0.0.1 by default -- localhost only, unless that machine and this script's --host differ
and the C++ side is rebuilt to bind 0.0.0.0). Which physical machine that is (the Franka
workstation itself, or a separate dedicated realtime PC) is NOT YET CONFIRMED -- check before
running. Start the controller first (see franka_control_suite/README.md for the runner command),
*then* this script.

Hardware-tested 2026-09-14 (men119 account): franka_control's cold-start crash (unsafe all-zero
ActionSubscriber default under ControlMode::ABSOLUTE -- see the vault note) and its move-to-rest
speed (0.5 -> 0.1) are both fixed as of commit 3cafad1. This script's own state-read dtype bug
(float32 -> float64, above) was caught on the first live connection attempt, not yet re-tested
after the fix. Also note: this protocol has no gripper channel in the reviewed files (7-float
pose only) -- gripper control (if needed) is a separate, not-yet-identified path (likely
libfranka's gripper API directly, or a separate ZMQ topic not seen in the files checked so far).
A/B buttons are wired to print a TODO instead of silently doing nothing.
"""
import argparse
import time

import numpy as np
import pygame
import zmq
from scipy.spatial.transform import Rotation as R

# ---- tunables -----------------------------------------------------------------------------
HZ = 30.0                       # matches the architecture's "~30 Hz policy/command" rate
DT = 1.0 / HZ
DEADZONE = 0.12
MAX_LIN_VEL = 0.15               # m/s at full stick deflection
MAX_ROT_VEL = 0.4                # rad/s at full stick/trigger deflection
CMD_PORT = 2069
STATE_PORT = 2096


def apply_deadzone(x: float, dz: float = DEADZONE) -> float:
    if abs(x) < dz:
        return 0.0
    return np.sign(x) * (abs(x) - dz) / (1.0 - dz)


def recv_ee_pose(sub_socket) -> tuple[np.ndarray, np.ndarray]:
    """Blocking receive of the latest EE state -> (xyz, quat_xyzw)."""
    msg = sub_socket.recv()
    mat = np.frombuffer(msg, dtype=np.float64).reshape(4, 4, order="F")
    xyz = mat[:3, -1].astype(np.float64)
    quat_xyzw = R.from_matrix(mat[:3, :3]).as_quat()  # scipy returns [x,y,z,w]
    return xyz, quat_xyzw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1", help="host running franka_control_suite's ZMQ sockets")
    ap.add_argument("--print-raw", action="store_true", help="print raw axis/button values, no ZMQ connection")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick/controller detected by pygame/SDL. Is the Xbox controller connected?")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using controller: {js.get_name()}  axes={js.get_numaxes()}  buttons={js.get_numbuttons()}")

    if args.print_raw:
        print("Printing raw axis/button values (Ctrl+C to stop). No ZMQ connection made.")
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
            buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
            print(f"axes={axes}  buttons={buttons}", end="\r")
            time.sleep(0.1)

    ctx = zmq.Context()
    cmd_pub = ctx.socket(zmq.PUB)
    cmd_pub.bind(f"tcp://{args.host}:{CMD_PORT}") if args.host == "127.0.0.1" else cmd_pub.connect(f"tcp://{args.host}:{CMD_PORT}")
    state_sub = ctx.socket(zmq.SUB)
    state_sub.setsockopt(zmq.CONFLATE, True)
    state_sub.connect(f"tcp://{args.host}:{STATE_PORT}")
    state_sub.setsockopt(zmq.SUBSCRIBE, b"")

    print(f"Connecting to franka_control_suite on {args.host} (cmd :{CMD_PORT}, state :{STATE_PORT})...")
    xyz, quat = recv_ee_pose(state_sub)
    print(f"Initial EE pose: xyz={np.round(xyz, 3)} quat_xyzw={np.round(quat, 3)}")
    print("Starting teleop loop — Ctrl+C to stop.")

    try:
        while True:
            t0 = time.time()
            pygame.event.pump()

            # Axis order confirmed via --print-raw on the actual hardware (Xbox Series X
            # controller, this SDL/pygame build): [LX, LY, LT, RX, RY, RT] -- triggers are
            # interleaved between the sticks here, NOT [LX, LY, RX, RY, LT, RT] as originally
            # assumed. Found 2026-09-14 when the right-stick/trigger channels didn't behave as
            # expected on the first live test (left stick was unaffected -- axes 0/1 were already
            # correct either way).
            lx = apply_deadzone(js.get_axis(0))
            ly = apply_deadzone(-js.get_axis(1))
            lt = (js.get_axis(2) + 1) / 2 if js.get_numaxes() > 2 else 0.0
            rx = apply_deadzone(js.get_axis(3))
            ry = apply_deadzone(-js.get_axis(4))
            rt = (js.get_axis(5) + 1) / 2 if js.get_numaxes() > 5 else 0.0
            lb = js.get_button(4) if js.get_numbuttons() > 4 else 0
            rb = js.get_button(5) if js.get_numbuttons() > 5 else 0
            btn_a = js.get_button(0)
            btn_b = js.get_button(1)
            btn_back = js.get_button(6) if js.get_numbuttons() > 6 else 0

            if btn_back:
                print("\nBack/Select pressed — stopping.")
                break
            if btn_a or btn_b:
                print(" [gripper control not wired yet — see module docstring]", end="\r")

            # --- read latest actual EE state (non-blocking; reuse last known pose if none ready) ---
            try:
                state_sub.setsockopt(zmq.RCVTIMEO, 0)
                xyz, quat = recv_ee_pose(state_sub)
            except zmq.Again:
                pass  # no fresher state this tick — keep integrating from our own last target

            # --- integrate this tick's delta on top of current state ---
            lin_delta = np.array([lx, ly, ry]) * MAX_LIN_VEL * DT
            roll = (rb - lb) * MAX_ROT_VEL * DT
            pitch = (rt - lt) * MAX_ROT_VEL * DT
            yaw = rx * MAX_ROT_VEL * DT

            xyz = xyz + lin_delta
            if abs(roll) + abs(pitch) + abs(yaw) > 1e-9:
                delta_rot = R.from_rotvec([roll, pitch, yaw])
                quat = (delta_rot * R.from_quat(quat)).as_quat()

            send_data = np.concatenate([xyz, quat]).astype(np.float64)
            cmd_pub.send(send_data.tobytes())

            elapsed = time.time() - t0
            time.sleep(max(0.0, DT - elapsed))
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
