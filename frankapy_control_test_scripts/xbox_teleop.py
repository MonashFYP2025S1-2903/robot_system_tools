"""
Xbox controller -> Franka EE-delta teleop, built directly on FrankaPy (no CPS repo, no IK code —
FrankaArm.goto_pose_delta() already does Cartesian impedance control via franka-interface).

Mapping:
    Left stick  X/Y  -> EE X/Y translation
    Right stick Y    -> EE Z translation
    Right stick X    -> EE yaw (rotation about world Z)
    LB/RB            -> EE roll  (- / +)
    LT/RT            -> EE pitch (- / +)   (triggers are analog 0..1 on most SDL mappings)
    A                -> close gripper
    B                -> open gripper
    Back/Select      -> quit

Run on the Franka workstation (same box/ROS network as `franka-interface`), e.g.:
    conda activate <frankapy env>
    python xbox_teleop.py

Related Obsidian vault note: Work_CSIRO/P_2026-08-28_TactileGap_PreferenceLearning/2026-09-11_OMX_Teleop_Franka_CPS_Code.md
(same trajectory-collection goal as the CPS OMX-leader / SpaceMouse paths documented there; this
script is the CPS-independent alternative, §12 "Gamepad EE control" in that note names the CPS
repo's own joystick_teleop.py — this reimplements the same idea without touching the CPS repo).

NOT YET TESTED ON HARDWARE. Two things to verify before trusting it on the real robot:
  1. Whether this FrankaPy version's `goto_pose_delta` streams smoothly when called back-to-back
     with short `duration`, or resets velocity to zero each call (a bit jerky either way — this
     script uses short overlapping-duration calls, the common pattern for FrankaPy teleop; if it's
     too jerky, look at FrankaPy's own dynamic/streaming skill examples instead of discrete calls).
  2. Confirm axis/button indices for YOUR controller as seen by SDL — printed at startup by
     `--print-raw`; Xbox controllers are consistent but pygame's SDL mapping can vary by OS/driver.
"""
import argparse
import time

import numpy as np
import pygame
from autolab_core import RigidTransform
from frankapy import FrankaArm

# ---- tunables -----------------------------------------------------------------------------
HZ = 20.0                      # control loop rate
DT = 1.0 / HZ
STEP_DURATION = 1.5 * DT       # FrankaPy motion duration per delta call — slightly > DT so calls overlap
DEADZONE = 0.12                # stick deflection below this = zero
MAX_LIN_VEL = 0.08              # m/s at full stick deflection
MAX_ROT_VEL = 0.4               # rad/s at full stick/trigger deflection
GRIPPER_DEBOUNCE_S = 0.5        # min time between gripper commands


def apply_deadzone(x: float, dz: float = DEADZONE) -> float:
    if abs(x) < dz:
        return 0.0
    # rescale so output is continuous from 0 at the deadzone edge to 1 at full deflection
    return np.sign(x) * (abs(x) - dz) / (1.0 - dz)


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Small-angle-safe axis-angle -> rotation matrix (Rodrigues)."""
    theta = np.linalg.norm(rotvec)
    if theta < 1e-8:
        return np.eye(3)
    axis = rotvec / theta
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--print-raw", action="store_true", help="print raw axis/button values, no robot connection")
    ap.add_argument("--no-gripper", action="store_true", help="skip FrankaArm(with_gripper=...) gripper init")
    args = ap.parse_args()

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick/controller detected by pygame/SDL. Is the Xbox controller connected?")
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using controller: {js.get_name()}  axes={js.get_numaxes()}  buttons={js.get_numbuttons()}")

    if args.print_raw:
        print("Printing raw axis/button values (Ctrl+C to stop). No robot connection made.")
        while True:
            pygame.event.pump()
            axes = [round(js.get_axis(i), 2) for i in range(js.get_numaxes())]
            buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
            print(f"axes={axes}  buttons={buttons}", end="\r")
            time.sleep(0.1)

    fa = FrankaArm(with_gripper=not args.no_gripper)
    print("FrankaArm connected. Starting teleop loop — Ctrl+C to stop.")

    last_gripper_cmd_t = 0.0
    gripper_closed = False

    try:
        while True:
            t0 = time.time()
            pygame.event.pump()

            # --- read axes (indices are the common SDL/XInput Xbox layout; verify with --print-raw) ---
            lx = apply_deadzone(js.get_axis(0))   # left stick X
            ly = apply_deadzone(-js.get_axis(1))  # left stick Y (SDL is inverted: up = negative)
            rx = apply_deadzone(js.get_axis(2))   # right stick X (some layouts: axis 3)
            ry = apply_deadzone(-js.get_axis(3))  # right stick Y (some layouts: axis 4)
            lt = (js.get_axis(4) + 1) / 2 if js.get_numaxes() > 4 else 0.0   # trigger: -1..1 -> 0..1
            rt = (js.get_axis(5) + 1) / 2 if js.get_numaxes() > 5 else 0.0
            lb = js.get_button(4) if js.get_numbuttons() > 4 else 0
            rb = js.get_button(5) if js.get_numbuttons() > 5 else 0
            btn_a = js.get_button(0)
            btn_b = js.get_button(1)
            btn_back = js.get_button(6) if js.get_numbuttons() > 6 else 0

            if btn_back:
                print("\nBack/Select pressed — stopping.")
                break

            # --- build delta pose for this tick ---
            lin = np.array([lx, ly, ry]) * MAX_LIN_VEL * DT           # x, y from left stick; z from right-stick Y
            roll = (rb - lb) * MAX_ROT_VEL * DT
            pitch = (rt - lt) * MAX_ROT_VEL * DT
            yaw = rx * MAX_ROT_VEL * DT
            rot = rotvec_to_matrix(np.array([roll, pitch, yaw]))

            moving = np.any(np.abs(lin) > 1e-6) or np.any(np.abs([roll, pitch, yaw]) > 1e-6)
            if moving:
                delta = RigidTransform(rotation=rot, translation=lin,
                                        from_frame="franka_tool", to_frame="franka_tool")
                fa.goto_pose_delta(delta, use_impedance=True, duration=STEP_DURATION, block=False)

            # --- gripper (debounced) ---
            now = time.time()
            if now - last_gripper_cmd_t > GRIPPER_DEBOUNCE_S:
                if btn_a and not gripper_closed:
                    fa.close_gripper()
                    gripper_closed = True
                    last_gripper_cmd_t = now
                elif btn_b and gripper_closed:
                    fa.open_gripper()
                    gripper_closed = False
                    last_gripper_cmd_t = now

            elapsed = time.time() - t0
            time.sleep(max(0.0, DT - elapsed))
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
