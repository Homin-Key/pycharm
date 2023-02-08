"""
RRR
"""

import numpy as np
from spatialmath.base import *
from spatialmath import *
from matplotlib import pyplot as plt

L1 = 0.2
L2 = 1
L3 = 0.8


def forkine(theta1, theta2, theta3):
    """
    Write this as part of ICE4
    """

    # Construct transformation matrices
    # Hint: Lecture 19 Slide 5
    # Screw_X(alpha, a) * Screw_Z(d, theta)
    # Use SE3, SE3.Rx, and SE3.Rz
    T1 = SE3.Rx(0) * SE3(0, 0, 0) * SE3(0, 0, L1) * SE3.Rz(theta1)   # rewrite this line
    T2 = SE3.Rx(90) * SE3(0, 0, 0) * SE3(0, 0, 0) * SE3.Rz(theta2)   # rewrite this line
    T3 = SE3.Rx(0) * SE3(L2, 0, 0) * SE3(0, 0, 0) * SE3.Rz(theta3)   # rewrite this line
    T4 = SE3.Rx(0) * SE3(L3, 0, 0) * SE3(0, 0, 0) * SE3.Rz(0)   # rewrite this line

    return T1*T2*T3*T4


def main():

    q = np.array(
        ((0, 0, 0),
         (90, 0, 0),
         (0, 90, 0),
         (0, 0, 90),
         (90, 90, 0),
         (0, 90, 90),
         (90, 0, 90),
         (90, 90, 90),
         (45, 0, 0),
         (0, 45, 0),
         (0, 0, 45))
    )*np.pi/180

    for joint_angles in q:

        # When specifying a list or tuple with * as an argument, it is unpacked,
        # and each element is passed to each argument.
        T = forkine(*joint_angles)
        T.printline()
        T.plot()
        plotvol3([-2.2, 2.2, -2.2, 2.2, -2.2, 2.2], grid=True)
        plt.show()


if __name__ == '__main__':
    main()
