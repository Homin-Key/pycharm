"""
RRR
"""

import roboticstoolbox as rtb
from numpy import pi
from spatialmath import *
from matplotlib import pyplot as plt
L1 = 193
L2 = 190
L3 = 60
L4 = 290
L5 = 75
L6 = 70


    #rtb.RevoluteMDH(d=L1),
    #rtb.RevoluteMDH(alpha=pi/2, offset=pi/2),
    #rtb.RevoluteMDH(a=L2),
    #rtb.RevoluteMDH(alpha=pi/2, a=L3, d=L4),
    #rtb.RevoluteMDH(alpha=-pi/2),
    #rtb.RevoluteMDH(alpha=pi/2)

def main():

    RPR = rtb.robot.DHRobot(
        [
        rtb.RevoluteMDH(d=L1),
        rtb.RevoluteMDH(alpha=pi/2, offset=pi/2),
        rtb.RevoluteMDH(a=L2),
        rtb.RevoluteMDH(alpha=pi/2, a=L3, d=L4),
        rtb.RevoluteMDH(alpha=-pi/2),
        rtb.RevoluteMDH(alpha=pi/2)
        ], name='RPR',      tool = SE3([0, 0 ,L5+L6]))
    #RPR.tool = SE3.Tz(L5+L6)
    print(RPR)
    print(RPR.tool)

    qz = [0, 0, 0, 0, 0, 0]

    #[0, pi/2, 0, 0, 0]
    # [0, 0, 0, 0, 0]
    #[0, pi/2, -pi/2, 0, 0]
    #[0, pi/2, pi/2, 0, 0]
    #[0, pi/2, 0, -pi/2, 0]
    #[0, pi/2, 0, pi/2, 0]
    #[0, pi/2, 0, 0, -pi/2]
    #[0, pi/2, 0, 0, pi/2]
    #[-pi/2, pi/2, -pi/2, 0, -pi/2]
    #
    T = RPR.fkine(qz)
    print(T)
    T.printline()
    RPR.plot(qz, block=True)


if __name__ == '__main__':
    main()
