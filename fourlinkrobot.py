"""
ECE487 Four-Link Manipulator
"""

import numpy as np
from numpy import pi
import roboticstoolbox as rtb
from spatialmath import SE3


class FourLinkRobot(rtb.DHRobot):

    def __init__(self, link1=1, link2=1, link3=1, link4=1):

        self.L1 = link1
        self.L2 = link2
        self.L3 = link3
        self.L4 = link4

        # Call superclass constructor.
        rtb.DHRobot.__init__(self,
            [rtb.RevoluteMDH(d=link1),
             rtb.RevoluteMDH(alpha=pi/2),
             rtb.RevoluteMDH(a=link2),
             rtb.RevoluteMDH(a=link3)],
            name="4DoF"
        )
        self.tool = SE3([link4, 0, 0])
        np.set_printoptions(precision=3, suppress=True)

    @staticmethod
    def normalize_angle(angle):
        """ Normalize an angle so that it is between -pi and pi.
        :param angle: Angle in radian in a range of [-2*pi, 2*pi]
        :return: Normalized angle.
        """
        if angle > pi:
            return angle - 2*pi
        elif angle < -pi:
            return angle + 2*pi
        else:
            return angle

    def invkine(self, pose):
        """ Returns the closed-form solutions to inverse kinematics.
        :param pose: 1x4 array for the pose (x,y,z,phi) of the tooltip.
        :return: a list of joint angles in radians.
        The list is an empty list if there exists no solution,
        a list of a 1x4 list if there exists a unique solution, and
        a list of two 1x4 lists if there exist 2 solutions.

        Example:
        pose = [px, py, pz, phi]
        robot = FourLinkRobot()
        thetas = robot.invkine(pose)
        """

        px = pose[0]
        py = pose[1]
        pz = pose[2]
        phi = pose[3]

        d = np.sqrt(px**2 + pz**2)

        # Write your code here
        if d > self.L2 + self.L3:
            return []
        elif d == self.L2 + self.L3:
            theta3 = np.arccos((px**2 + pz**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            alpha = np.arccos((d**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * d))
            beta = np.arctan2(pz, px)
            theta2 = beta - (theta3 * alpha)
            theta1 = np.arctan2(py, px)
            theta4 = phi - beta - theta3
            return [[theta1, beta, 0, theta4]]
        elif d == 0:
            theta3 = np.arccos((px**2 + pz**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            theta1 = np.arctan2(py, px)
            theta4 = phi - 0 - theta3
            return[[theta1, 0, theta3, theta4], [theta1, 0, -theta3, theta4]]
        elif d < self.L2 + self.L3:
            theta3 = np.arccos((px**2 + pz**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
            alpha = np.arccos((d**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * d))
            beta = np.arctan2(pz, px)
            theta2one = beta - alpha
            theta2two = beta + alpha
            theta1 = np.arctan2(py, px)
            theta4one = phi - theta2one - theta3
            theta4two = phi - theta2two - theta3
            #theta2 = beta - (theta3 * alpha)
            return [[theta1, theta2one, theta3, theta4one], [theta1, theta2two, -theta3, theta4two]]

def main():

    L1 = 0
    L2 = 1
    L3 = 1
    L4 = 0.5
    robot = FourLinkRobot(L1, L2, L3, L4)

    poses = list()
    poses.append((2.5, 0, 0, 0))  # angles: ypr/zyx
    poses.append((2, 0, 0.5, pi/2))
    poses.append((0, 0, 2.5, pi/2))
    poses.append((1, 0, 0.5, -pi/2))
    poses.append((0, 0.5, 0, 0))  # [0, pi/2, pi/2, 0, 0, 0])
    poses.append((2.5*np.cos(pi/4), 0, 2.5*np.sin(pi/4), pi/4))
    poses.append((2.5*np.cos(pi/4), 2.5*np.sin(pi/4), 0, 0))
    poses.append((-2.5, 0, 0, 0))
    poses.append((-2, 0, 0.5, pi/2))
    poses.append((0, -2.5, 0, 0))
    poses.append((2, 1, 0.5, pi))
    poses.append((2, 2, 0, pi/2))

    np.set_printoptions(precision=3, suppress=True)

    for pose in poses:
        print(" ")
        print("===================================================")
        print(f"Pose: t = {pose[:3]}; phi = {np.rad2deg(pose[3:])}")
        joint_angles = robot.invkine(pose)

        if len(joint_angles) == 0:
            print("Out of workspace! No solution exists.")
        else:
            print(np.rad2deg(joint_angles))

            for angles in joint_angles:
                T = robot.fkine(angles)
                T.printline()
                # Caution: plot sometimes does not depict the tooltip
                robot.plot(angles, block=True)


if __name__ == '__main__':
    main()
