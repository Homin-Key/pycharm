"""
Extract Euler Angles
"""

import numpy as np


def extract_euler_angles(Rzyx: np.array):
    """
    Finds the ZYX Euler angles (psi, theta, phi) from rotation matrix R.
    If theta = +/-90 deg, the solutions degenerate. In this case, this
    function returns psi = 0, theta = +/-90 deg, and phi.
    :param Rzyx: 3x3 numpy array representing rotation in 3-D.
    :return: a set of ZYX Euler angels (psi, theta, phi) in radians, in the range [-pi, pi].
    If the solutions degenerate, it returns one set of Euler angles.

    Example:
    thetas = extract_euler_angles(Rzyx)
    """

    assert Rzyx.shape == (3, 3), "Rzyx must be a 3x3 np array"

    r11 = Rzyx[0, 0]
    r12 = Rzyx[0, 1]
    r13 = Rzyx[0, 2]
    r21 = Rzyx[1, 0]
    r22 = Rzyx[1, 1]
    r23 = Rzyx[1, 2]
    r31 = Rzyx[2, 0]
    r32 = Rzyx[2, 1]
    r33 = Rzyx[2, 2]

    theta1 = np.arctan2(-r31, np.sqrt((r32*r32) + (r33 * r33)))
    theta2 = np.arctan2(-r31, -np.sqrt((r32*r32) + (r33 * r33)))
    if np.isclose(theta1, np.pi/2) or np.isclose(theta2, np.pi/2):
        psi = 0
        phi = np.arctan2(r12, r22)
        return [[psi, theta1, phi]]
    elif np.isclose(theta1, -np.pi/2) or np.isclose(theta2, -np.pi/2):
        psi = 0
        phi = np.arctan2(-r12, r22)
        return [[psi, theta1, phi]]
    else:
        psi1 = np.arctan2(r21/np.cos(theta1), r11/np.cos(theta1))
        psi2 = np.arctan2(r21/np.cos(theta2), r11/np.cos(theta2))
        phi1 = np.arctan2(r32/np.cos(theta1), r33/np.cos(theta1))
        phi2 = np.arctan2(r32/np.cos(theta2), r33/np.cos(theta2))
        return [[psi1, theta1, phi1], [psi2, theta2, phi2]]


def main():

    # Add more test angles
    test_angles = np.array([[0, 45, 0],
                            [0, 0, 90],
                            [0, 0, 0],
                            [90, 90, 90],
                            [45, 45, 45]]
                            ) * (np.pi/180)

    for angles in test_angles:
        alpha = angles[0] #psi
        beta = angles[1] #theta
        gamma = angles[2] #phi
        Rz = np.array([[np.cos(alpha), -np.sin(alpha), 0], [np.sin(alpha), np.cos(alpha), 0], [0, 0, 1]])
        Ry = np.array([[np.cos(beta), 0, np.sin(beta)], [0, 1, 0], [-np.sin(beta), 0, np.cos(beta)]])
        Rx = np.array([[1, 0, 0], [0, np.cos(gamma), -np.sin(gamma)], [0, np.sin(gamma), np.cos(gamma)]])
        matrix = Rz @ Ry @ Rx
        angles = extract_euler_angles(matrix)
        for angle in angles:
            alpha1 = angle[0]
            beta1 = angle[1]
            gamma1 = angle[2]
            Rz1 = np.array([[np.cos(alpha1), -np.sin(alpha1), 0], [np.sin(alpha1), np.cos(alpha1), 0], [0, 0, 1]])
            Ry1 = np.array([[np.cos(beta1), 0, np.sin(beta1)], [0, 1, 0], [-np.sin(beta1), 0, np.cos(beta1)]])
            Rx1 = np.array([[1, 0, 0], [0, np.cos(gamma1), -np.sin(gamma1)], [0, np.sin(gamma1), np.cos(gamma1)]])
            matrix1 = Rz1 @ Ry1 @ Rx1
            angles1 = extract_euler_angles(matrix1)
        if np.all(np.isclose(matrix, matrix1,)):
            print(np.array(angles) * (180/np.pi))#

        else:
            print("Error")
        # Add your code below


if __name__ == '__main__':
    main()
