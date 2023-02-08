"""
Two-Link Planar Arm

required packages:
- numpy
- roboticstoolbox
- spatialmath
"""

import numpy as np
from numpy import pi
import roboticstoolbox as rtb
from spatialmath import SE3


class TwoLinkRobot(rtb.DHRobot):

    def __init__(self, link2=1, link3=1):

        self.L2 = link2
        self.L3 = link3

        # Call superclass constructor.
        rtb.DHRobot.__init__(self,
            [rtb.RevoluteMDH(),
             rtb.RevoluteMDH(a=self.L2)],
            name="xArm", tool=SE3([self.L3, 0, 0])
        )

        np.set_printoptions(precision=3, suppress=True)

    def invkine(self, pose):
        """ Returns the closed-form solutions to inverse kinematics.
        :param pose: 1x2 array for the 2-D (x-z plane) position of the tooltip.
        :return: an array of joint angles in radians.
        The array is an empty list if there exists no solution,
        a list of a 1x2 list if there exists a unique solution, and
        a list of two 1x2 lists if there exist 2 solutions.

        Example:
        pose = [px, pz]
        robot = TwoLinkRobot()
        thetas = robot.invkine(pose)

        If no solution exists, thetas = []
        If one solution exists, thetas = [[theta2, theta3]]
        If two solution exist, thetas = [[theta2_1, theta3_1], [theta2_2, theta3_2]]

        """

        x = pose[0]
        z = pose[1]

        d = np.sqrt(x**2 + z**2)

        # 1. Return an empty list if there exists no solution
        # 2. Return a list of one 1x2 list if there exists one solution
        # 3. Return a list of two 1x2 lists if there exist two solutions
        # write your code here


        if d > self.L2 + self.L3:
            return []
        elif d == self.L2 + self.L3:
            theta3 = np.arccos((x**2 + z**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            alpha = np.arccos((d**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * d))
            beta = np.arctan2(z, x)
            theta2 = beta - (theta3 * alpha)
            return [[beta, 0]]
        elif d == 0:
            theta3 = np.arccos((x**2 + z**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            return[[0, theta3], [0, -theta3]]
        elif d < self.L2 + self.L3:
            theta3 = np.arccos((x**2 + z**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            alpha = np.arccos((d**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * d))
            beta = np.arctan2(z, x)
            theta2one = beta - alpha
            theta2two = beta + alpha
            #theta2 = beta - (theta3 * alpha)
            return [[theta2one, theta3], [theta2two, -theta3]]







def main():

    robot = TwoLinkRobot()

    poses = ((0, 2),
             (0, -2),
             (0, 0),
             (1, 1),
             (1, -1),
             (-1, 1),
             (-1, -1),
             (3,  0))

    for pose in poses:
        joint_angles = robot.invkine(pose)
        print('======================')
        print('desired pose: ', pose)

        if len(joint_angles) == 0:
            print("Out of workspace! No solution exists.")
        else:
            for angles in joint_angles:
                T = robot.fkine(angles)
                print('joint angle: ', np.rad2deg(angles))
                print('pose:', end=' ')
                T.printline()


if __name__ == '__main__':
    main()
