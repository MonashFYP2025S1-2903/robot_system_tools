import numpy as np, sys
from panda_reach import *
R_down = np.diag([1.0, -1.0, -1.0])
q_ready = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
rng = np.random.default_rng(0)
seeds = [q_ready] + [np.clip(q_ready + rng.normal(0, 0.35, 7), QMIN + 0.1, QMAX - 0.1) for _ in range(3)]
xs = np.arange(0.25, 0.851, 0.05)
ys = np.arange(-0.5, 0.501, 0.1)
zs = [0.05, 0.15, 0.25, 0.35, 0.45]
best = {}
for z in zs:
    print(f"\nz = {z:.2f} m  (rows: y from +0.5 robot-left to -0.5; cols: x from {xs[0]:.2f} to {xs[-1]:.2f} m)")
    print("      " + " ".join(f"{x:4.2f}" for x in xs))
    for y in ys[::-1]:
        row = []
        for x in xs:
            bm = -9; bq = None
            for sd in seeds:
                q, ok = ik(np.array([x, y, z]), R_down, sd)
                if ok and margin(q) > bm:
                    bm, bq = margin(q), q
            best[(round(x, 2), round(y, 2), z)] = (bm, bq)
            row.append(" .  " if bm >= 0.30 else (" +  " if bm >= 0.15 else (" -  " if bm >= 0.0 else " x  ")))
        print(f"{y:+.1f}  " + "".join(row))
print("\nlegend: '.' every joint >= 0.30 rad from its limit, '+' >= 0.15, '-' >= 0 (tight), 'x' no solution with gripper straight down")
import pickle; pickle.dump(best, open("reach_best.pkl", "wb"))
