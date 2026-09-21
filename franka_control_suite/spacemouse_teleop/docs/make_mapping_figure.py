import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, FancyBboxPatch

fig, axes = plt.subplots(1, 2, figsize=(12, 6.4), dpi=140)
GREY, BLUE, ORANGE, INK = "#d9dde3", "#1f6fb5", "#d9822b", "#1b1f24"

def panel(ax, title, subtitle, items, colour):
    ax.set_xlim(-3.2, 3.2); ax.set_ylim(-3.75, 3.2); ax.set_aspect("equal"); ax.axis("off")
    ax.text(0, 3.0, title, ha="center", va="center", fontsize=14, fontweight="bold", color=INK)
    ax.text(0, 2.6, subtitle, ha="center", va="center", fontsize=9.5, color="#4a5560")
    ax.add_patch(Circle((0, 0), 0.62, fc=GREY, ec=INK, lw=1.6))
    ax.text(0, 0, "cap\n(top view)", ha="center", va="center", fontsize=8, color=INK)
    ax.text(0, -3.45, "(you sit at the bottom of this diagram, facing up)", ha="center", va="center", fontsize=8, color="#7a838d", style="italic")
    for (dx, dy, head, body) in items:
        ax.add_patch(FancyArrowPatch((dx*0.7, dy*0.7), (dx*1.55, dy*1.55), arrowstyle="-|>", mutation_scale=18, color=colour, lw=2.4))
        tx, ty = dx*2.05, dy*1.95
        ax.text(tx, ty + 0.16, head, ha="center", va="center", fontsize=10, fontweight="bold", color=colour)
        ax.text(tx, ty - 0.22, body, ha="center", va="center", fontsize=8.6, color=INK, linespacing=1.25)

panel(axes[0], "MOVE mode (default)", "cap PUSH / PULL / TWIST  ->  gripper moves", [
    (0, 1, "push FORWARD", "gripper moves to the\nROBOT'S LEFT"),
    (0, -1, "pull BACK", "gripper moves to the\nROBOT'S RIGHT"),
    (-1, 0, "push LEFT", "gripper moves TOWARD\nthe robot base"),
    (1, 0, "push RIGHT", "gripper moves AWAY\nfrom the robot base"),
], BLUE)
axes[0].text(0, -2.85, "push DOWN / pull UP  ->  gripper moves down / up\ntwist the cap (yaw, needs --rot)  ->  gripper yaws the same way", ha="center", va="center", fontsize=9, color=INK, linespacing=1.4)

panel(axes[1], "TILT mode (press BOTH buttons)", "cap TILT  ->  gripper rotates, translation is OFF", [
    (0, 1, "tilt FORWARD", "gripper pitches:\ntip swings toward YOU"),
    (0, -1, "tilt BACK", "gripper pitches:\ntip swings away from you"),
    (-1, 0, "tilt LEFT", "gripper rolls:\ntip swings to your RIGHT"),
    (1, 0, "tilt RIGHT", "gripper rolls:\ntip swings to your LEFT"),
], ORANGE)
axes[1].text(0, -2.85, "press both buttons again to go back to MOVE mode\ntwist still yaws the gripper in this mode", ha="center", va="center", fontsize=9, color=INK, linespacing=1.4)

fig.text(0.5, 0.045, "LEFT button HELD = close gripper     RIGHT button HELD = open gripper     BOTH together = switch MOVE / TILT     start with the cap at rest (arms after 0.3 s)",
         ha="center", va="center", fontsize=9.2, color=INK,
         bbox=dict(boxstyle="round,pad=0.5", fc="#f3f5f7", ec="#b8bfc7"))
plt.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.10, wspace=0.02)
fig.savefig("spacemouse_franka_mapping.png", facecolor="white")
print("saved")
