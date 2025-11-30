#!/usr/bin/env python3
import numpy as np


class Planar3DoFArm:
    """
    Simple 3-DoF planar arm model.

    q = [q1, q2, q3]  (joint angles in radians)
    l = [l1, l2, l3]  (link lengths in meters)
    """

    def __init__(self, l1=0.3, l2=0.3, l3=0.2):
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3

    def forward_kinematics(self, q):
        """
        Compute end-effector position (x, y) and orientation alpha.

        q: iterable with 3 joint angles [q1, q2, q3] in radians.
        Returns: (x, y, alpha)
        """
        q1, q2, q3 = q

        # Cumulative angles
        th1 = q1
        th2 = q1 + q2
        th3 = q1 + q2 + q3

        x = (
            self.l1 * np.cos(th1)
            + self.l2 * np.cos(th2)
            + self.l3 * np.cos(th3)
        )
        y = (
            self.l1 * np.sin(th1)
            + self.l2 * np.sin(th2)
            + self.l3 * np.sin(th3)
        )
        alpha = th3  # end-effector orientation

        return x, y, alpha

    def jacobian(self, q):
        """
        Compute the 3x3 planar Jacobian J(q) for [Fx, Fy, tau_z].

        q: [q1, q2, q3]
        Returns: 3x3 numpy array J
        """
        q1, q2, q3 = q
        th1 = q1
        th2 = q1 + q2
        th3 = q1 + q2 + q3

        # Partial derivatives for x
        dx_dq1 = -self.l1 * np.sin(th1) - self.l2 * np.sin(th2) - self.l3 * np.sin(th3)
        dx_dq2 = -self.l2 * np.sin(th2) - self.l3 * np.sin(th3)
        dx_dq3 = -self.l3 * np.sin(th3)

        # Partial derivatives for y
        dy_dq1 = self.l1 * np.cos(th1) + self.l2 * np.cos(th2) + self.l3 * np.cos(th3)
        dy_dq2 = self.l2 * np.cos(th2) + self.l3 * np.cos(th3)
        dy_dq3 = self.l3 * np.cos(th3)

        # Orientation alpha = q1 + q2 + q3
        dalpha_dq1 = 1.0
        dalpha_dq2 = 1.0
        dalpha_dq3 = 1.0

        J = np.array(
            [
                [dx_dq1, dx_dq2, dx_dq3],
                [dy_dq1, dy_dq2, dy_dq3],
                [dalpha_dq1, dalpha_dq2, dalpha_dq3],
            ]
        )

        return J


if __name__ == "__main__":
    arm = Planar3DoFArm()
    q_test = [0.0, np.pi / 4, -np.pi / 6]
    x, y, alpha = arm.forward_kinematics(q_test)
    print("FK test: x =", x, " y =", y, " alpha =", alpha)
    J = arm.jacobian(q_test)
    print("Jacobian:\n", J)

