import numpy as np

# link lengths (you can change if you want)
L1 = 0.4
L2 = 0.3
L3 = 0.2

def forward_kinematics(q):
    """
    q: [q1, q2, q3] in radians
    returns (x, y, alpha)
    alpha = end-effector orientation
    """
    q1, q2, q3 = q
    a1 = q1
    a2 = q1 + q2
    a3 = q1 + q2 + q3

    x = (L1 * np.cos(a1) +
         L2 * np.cos(a2) +
         L3 * np.cos(a3))
    y = (L1 * np.sin(a1) +
         L2 * np.sin(a2) +
         L3 * np.sin(a3))
    alpha = a3
    return x, y, alpha


def inverse_kinematics(x, y, alpha):
    """
    Simple planar 3R IK.
    Returns one solution (q1, q2, q3) in radians.
    """
    # wrist position (end of link 2)
    wx = x - L3 * np.cos(alpha)
    wy = y - L3 * np.sin(alpha)

    r2 = wx**2 + wy**2
    c2 = (r2 - L1**2 - L2**2) / (2 * L1 * L2)

    # clamp numerical errors
    c2 = np.clip(c2, -1.0, 1.0)

    # elbow-down solution
    s2 = np.sqrt(1.0 - c2**2)
    q2 = np.arctan2(s2, c2)

    k1 = L1 + L2 * c2
    k2 = L2 * s2
    q1 = np.arctan2(wy, wx) - np.arctan2(k2, k1)

    q3 = alpha - q1 - q2
    return np.array([q1, q2, q3])

def jacobian(q):
    q1, q2, q3 = q

    a1 = q1
    a2 = q1 + q2
    a3 = q1 + q2 + q3

    J11 = -L1*np.sin(a1) - L2*np.sin(a2) - L3*np.sin(a3)
    J12 = -L2*np.sin(a2) - L3*np.sin(a3)
    J13 = -L3*np.sin(a3)

    J21 =  L1*np.cos(a1) + L2*np.cos(a2) + L3*np.cos(a3)
    J22 =  L2*np.cos(a2) + L3*np.cos(a3)
    J23 =  L3*np.cos(a3)

    # orientation derivative
    J31 = 1.0
    J32 = 1.0
    J33 = 1.0

    J = np.array([
        [J11, J12, J13],
        [J21, J22, J23],
        [J31, J32, J33]
    ])

    return J


def torque_from_force(q, F):
    """
    q : [q1, q2, q3]
    F : [Fx, Fy, Mz]
    Computes tau = J^T * F
    """
    J = jacobian(q)
    return J.T @ F

if __name__ == "__main__":
    # quick FK/IK round-trip test
    q = np.deg2rad([10, 20, -15])
    x, y, a = forward_kinematics(q)
    q_sol = inverse_kinematics(x, y, a)
    print("q original:", q)
    print("q solved  :", q_sol)

    # simple torque test for Task 3: tau = J^T F
    F = np.array([5.0, 0.0, 0.0])   # Fx, Fy, Mz
    tau = torque_from_force(q, F)
    print("Force F:", F)
    print("Tau = J^T F:", tau)
