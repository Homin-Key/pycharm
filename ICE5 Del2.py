"""
RRR
"""

import roboticstoolbox as rtb
from numpy import pi
from spatialmath import *
from matplotlib import pyplot as plt
Bo = 0.090
L1 = 0
L2 = 0.105
L3 = 0.088
L4 = 0.170


    #rtb.RevoluteMDH(d=L1),
    #rtb.RevoluteMDH(alpha=pi/2, offset=pi/2),
    #rtb.RevoluteMDH(a=L2),
    #rtb.RevoluteMDH(alpha=pi/2, a=L3, d=L4),
    #rtb.RevoluteMDH(alpha=-pi/2),
    #rtb.RevoluteMDH(alpha=pi/2)

def main():

    RPR = rtb.robot.DHRobot(
        [
            rtb.RevoluteMDH(),
            rtb.RevoluteMDH(d=Bo),
            rtb.RevoluteMDH(a=L2),
            rtb.RevoluteMDH(a=L3),
            rtb.RevoluteMDH(d=L4),
        ], name='RPR')       #tool = SE3([0, 0 ,L5+L6]))
    #RPR.tool = SE3.Tz(L5+L6)
    print(RPR)
    print(RPR.tool)

    qz = [pi/2, pi/2, pi/2, 0, pi/2]

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
