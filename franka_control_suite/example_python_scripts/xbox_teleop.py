"""
Xbox controller -> Franka EE-delta teleop, via franka_control_suite's ZMQ interface
(NOT FrankaPy — see the vault note below for why that path is explicitly avoided here).

Protocol -- confirmed by reading the actual runner (src/runners/runner.cpp, the `franka_control`
executable), not just the example_python_scripts/example_CartesianMotionControl.py reference
script (whose float32 state dtype turned out to be wrong for this runner):
    Command: ZMQ PUB  tcp://127.0.0.1:2069 -> 8x float64 [x,y,z,qx,qy,qz,qw,gripper], an ABSOLUTE
             target pose plus a single gripper scalar (CommsDataType::POSE_QUAT_GRIPPER). The C++
             side runs InverseKinematics(M_P_PSEUDO_INVERSE) for the arm -- genuine differential IK
             (Jacobian pseudo-inverse -> joint *velocity*), NOT Cartesian impedance control (a
             CartesianImpedance controller class exists in the source tree but no executable is
             wired to it). The gripper scalar follows IsaacLab's mdp.BinaryJointPositionActionCfg
             sign convention (negative = close, non-negative = open -- see camera_wrist_demo.py's
             GRIPPER_CLOSE_ACTION = -1.0 in men119-isaaclabextensioncsirohri), read on the C++ side
             by a dedicated thread calling franka::Gripper::grasp()/move() directly -- same pattern
             as the already-proven joint_pos_runner.cpp used for the real cube-lift policy
             deployment (force-limited grasp at 20N, stops on contact, not a blind position close).
             Continuously publishing an updated absolute target *is* the streaming teleop interface.
    State:   ZMQ SUB  tcp://127.0.0.1:2096, CONFLATE=True -> 16x float64 (NOT float32 -- confirmed
             against StatePublisher::writeMessage's std::vector<double>), the current EE pose as a
             column-major 4x4 homogeneous transform.

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
and the C++ side is rebuilt to bind 0.0.0.0) -- confirmed to be the Franka workstation itself.
Start the controller first (see franka_control_suite/README.md for the runner command), *then*
this script.

Hardware-tested end-to-end on the Franka workstation (men119 account): cold-start crash, move
speed, state dtype, right-stick/trigger axis mapping, and realtime-scheduling permission (needs
`sudo setcap cap_sys_nice+ep` on the built `franka_control` binary -- re-run after every rebuild,
including after adding the gripper channel) were all found and fixed via live testing. A/B now
send the gripper scalar instead of printing a TODO; the runner-side grasp/release logic itself is
carried over from joint_pos_runner.cpp's proven pattern but the *combination* with this IK runner
is new -- treat the first gripper-enabled test as exactly that, not a confirmed-safe rerun.
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
DEFAULT_MAX_LIN_VEL = 1.5       # m/s at full stick deflection -- override with --max-lin-vel
DEFAULT_MAX_ROT_VEL = 2.0       # rad/s at full stick/trigger deflection -- override with --max-rot-vel
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
    ap.add_argument("--max-lin-vel", type=float, default=DEFAULT_MAX_LIN_VEL, help="m/s at full stick deflection")
    ap.add_argument("--max-rot-vel", type=float, default=DEFAULT_MAX_ROT_VEL, help="rad/s at full stick/trigger deflection")
    args = ap.parse_args()
    max_lin_vel = args.max_lin_vel
    max_rot_vel = args.max_rot_vel

    print("=" * 60)
    print("Xbox -> Franka EE-delta teleop")
    print(f"  max_lin_vel = {max_lin_vel} m/s   (translation, left stick + right stick Y)")
    print(f"  max_rot_vel = {max_rot_vel} rad/s (rotation: LB/RB=roll, LT/RT=pitch, right stick X=yaw)")
    print(f"  deadzone    = {DEADZONE}")
    print(f"  rate        = {HZ} Hz")
    print(f"  host        = {args.host} (cmd :{CMD_PORT}, state :{STATE_PORT})")
    print("  Override speed with --max-lin-vel / --max-rot-vel. Gripper: A=close, B=open.")
    print("=" * 60)

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

    gripper_cmd = 1.0  # open -- matches runner.cpp's primed default; A=close, B=open

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
            if btn_a:
                gripper_cmd = -1.0  # close (matches IsaacLab's negative=close convention)
            elif btn_b:
                gripper_cmd = 1.0   # open

            # --- read latest actual EE state (non-blocking; reuse last known pose if none ready) ---
            try:
                state_sub.setsockopt(zmq.RCVTIMEO, 0)
                xyz, quat = recv_ee_pose(state_sub)
            except zmq.Again:
                pass  # no fresher state this tick — keep integrating from our own last target

            # --- integrate this tick's delta on top of current state ---
            lin_delta = np.array([lx, ly, ry]) * max_lin_vel * DT
            roll = (rb - lb) * max_rot_vel * DT
            pitch = (rt - lt) * max_rot_vel * DT
            yaw = rx * max_rot_vel * DT

            xyz = xyz + lin_delta
            if abs(roll) + abs(pitch) + abs(yaw) > 1e-9:
                delta_rot = R.from_rotvec([roll, pitch, yaw])
                quat = (delta_rot * R.from_quat(quat)).as_quat()

            send_data = np.concatenate([xyz, quat, [gripper_cmd]]).astype(np.float64)
            cmd_pub.send(send_data.tobytes())

            elapsed = time.time() - t0
            time.sleep(max(0.0, DT - elapsed))
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
