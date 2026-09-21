"""Panda forward kinematics + numeric IK: which gripper positions (gripper pointing down) are reachable
with every joint at least `margin` rad inside its limits. Validated against a real O_T_EE reading."""
import numpy as np

QMIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
QMAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
A = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
D = [0.333, 0, 0.316, 0, 0.384, 0, 0]
AL = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, np.pi/2, np.pi/2]

def dh(a, d, al, th):
    ca, sa, ct, st = np.cos(al), np.sin(al), np.cos(th), np.sin(th)
    return np.array([[ct, -st, 0, a], [st*ca, ct*ca, -sa, -d*sa], [st*sa, ct*sa, ca, d*ca], [0, 0, 0, 1]])

def fk(q):
    T = np.eye(4)
    for i in range(7):
        T = T @ dh(A[i], D[i], AL[i], q[i])
    T = T @ dh(0, 0.107, 0, 0)
    c, s = np.cos(-np.pi/4), np.sin(-np.pi/4)
    T = T @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0.1034], [0, 0, 0, 1]])
    return T

def fk_chain(q):
    T = np.eye(4); zs = []; ps = []
    for i in range(7):
        T = T @ dh(A[i], D[i], AL[i], q[i])
        zs.append(T[:3, 2].copy()); ps.append(T[:3, 3].copy())
    T = T @ dh(0, 0.107, 0, 0)
    c, s = np.cos(-np.pi/4), np.sin(-np.pi/4)
    T = T @ np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0.1034], [0, 0, 0, 1]])
    return T, zs, ps

def jac_num(q, eps=1e-6):
    T, zs, ps = fk_chain(q)
    J = np.zeros((6, 7))
    for i in range(7):
        J[:3, i] = np.cross(zs[i], T[:3, 3] - ps[i])
        J[3:, i] = zs[i]
    return J

def ik(p, R, seed, iters=200, lam=0.02):
    q = seed.copy()
    for _ in range(iters):
        T = fk(q)
        ep = p - T[:3, 3]
        dR = R @ T[:3, :3].T
        eo = 0.5 * np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
        e = np.concatenate([ep, eo])
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            return q, True
        J = jac_num(q)
        # secondary task: pull toward joint-range mid to keep margins
        mid = (QMIN + QMAX) / 2
        Jp = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(6))
        dq = Jp @ e + (np.eye(7) - Jp @ J) @ (0.05 * (mid - q))
        q = np.clip(q + 0.5 * dq, QMIN - 0.05, QMAX + 0.05)
    return q, False

def margin(q):
    return float(np.min(np.minimum(q - QMIN, QMAX - q)))

if __name__ == "__main__":
    qcur = np.array([0.109, 0.298, -0.176, -2.224, 0.068, 3.748, 0.786])
    T = fk(qcur)
    print("FK of last status q:", np.round(T[:3, 3], 4), "(robot reported 0.6872 -0.0525 0.3609)")
    print("EE z-axis (gripper pointing) in base frame:", np.round(T[:3, 2], 3))
