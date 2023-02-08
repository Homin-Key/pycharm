"""
ECE487 Lab2
"""

# need to install pyserial package.
from serial.tools import list_ports
import serial
import numpy as np
from numpy import pi
from time import sleep
import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import SE3
import logging
from typing import NamedTuple
from spatialmath import *


"""
Naming convention
servo angles: -90 deg - +90 deg
servo positions: 500 - 2500
joint angles: theta1, theta2, ...
"""


class Pose(NamedTuple):
    id: int
    x: float
    y: float
    z: float
    phi: float
    psi: float


class Tag(NamedTuple):
    """ Data structure for AprilTag
    """
    id: int         # Tag ID
    x: float        # x position wrt Base frame
    y: float        # y position wrt Base frame
    z: float        # z position wrt Base frame
    cam_x: float    # x position wrt Camera frame
    cam_y: float    # x position wrt Camera frame
    cam_z: float    # x position wrt Camera frame


class XArm(rtb.DHRobot):

    HEADER = b'\x55'b'\x55'
    COMMAND = 3

    def __init__(self, simulation_only=False):
        self.camera_dist_offset = 0.037  # L4 - camera_distance

        self.mvcam = None
        self.curr_joint_angles = None

        # dictionary for block locations
        # for a short tutorial for Python dictionary, visit https://www.w3schools.com/python/python_dictionaries.asp
        self.block_locations = dict()

        self.B0 = 0.090
        self.L1 = 0.010     # self.L1 = 0.010 for inverse kinematics
        self.L2 = 0.105
        self.L3 = 0.088
        self.L4 = 0.170

        self.camera_dist_offset = 0.037  # L4 - camera_distance
        self.gripper_close_pos = 2200   # close position
        self.gripper_open_pos = 1700  # open position
        self.ordered_joints = ('base', 'shoulder', 'elbow', 'wpitch', 'wroll')

        # alpha = direction * theta + offset
        # where alpha is the servo angle and theta is the joint angle.
        self.joint_servo_offset = (0, pi/2, pi/4, 0, 0)  # angle offset between joint and servo
        self.joint_servo_direction = (1, -1, 1, -1, 1)   # 1 or -1

        self.max_joint_angle = (pi/2, pi, pi/4, pi/2, pi/2)
        self.min_joint_angle = (-pi/2, 0, -3*pi/4, -pi/2, -pi/2)

        self.plot_axes_limits = [-0.38, 0.38, -0.38, 0.38, 0, 0.47]

        np.set_printoptions(precision=3, suppress=True)

        # Update: super().__init__(...) to rtb.DHRobot.__init__(...)
        rtb.DHRobot.__init__(self,
            [rtb.RevoluteMDH(d=self.B0),
             rtb.RevoluteMDH(alpha=pi/2, a=self.L1),
             rtb.RevoluteMDH(a=self.L2),
             rtb.RevoluteMDH(a=self.L3, offset=-pi/2),
             rtb.RevoluteMDH(alpha=-pi/2, d=self.L4)],
            name="xArm"
        )

        self.comm = None
        self._simulation_only = simulation_only

        # ADD this for OpenMV Cam
        self.mvcam = None

        self.curr_joint_angles = None

        # dictionary for block locations
        # for short tutorial for Python dictionaries, visit https://www.w3schools.com/python/python_dictionaries.asp
        self.block_locations = dict()

    def connect(self):

        if self._simulation_only:
            print("Simulation-Only mode is enabled.")
            print("The robot is not connected.")
            return

        if isinstance(self.comm, serial.Serial):
            print(f"The robot is already connected to {self.comm.port}")
            return

        ports = list_ports.comports()
        comport = None

        for port, desc, hwid in sorted(ports):
            if ('Silicon Labs CP210x' in desc) or ('Prolific USB-to-Serial' in desc):
                comport = port
                print("Info: The robot is connecting to")
                print("{}: {} [{}]".format(port, desc, hwid))
                break

        if comport is None:
            print("Error: Serial port is not available.")
            print("The robot is not connected.")
            self._simulation_only = True
            return

        # A serial port is found.  Try to connect.
        try:
            self.comm = serial.Serial(
                port=comport,
                baudrate=9600,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
            )
            print(f"Info: The robot is connected to {comport}")

        except serial.SerialException:
            print("Error: Serial port is not available.")
            print("The robot is not connected.")
            print("Simulation-Only mode is enabled.")
            self._simulation_only = True

    def disconnect(self):
        if self.comm is not None:
            self.comm.close()

    def connect_mvcam(self):

        if isinstance(self.mvcam, serial.Serial):
            print(f"The robot is already connected to {self.comm.port}")
            return

        ports = list_ports.comports()
        comport = None

        for port, desc, hwid in sorted(ports):
            if 'USB Serial Device' in desc:
                comport = port
                print("Info: OpenMV Cam is connected to")
                print("{}: {} [{}]".format(port, desc, hwid))
                break

        if comport is None:
            print("Error: Serial port for OpenMV is not available.")
            print("OpenMV Cam is not connected.")
            return

        # A serial port is found.  Try to connect.
        try:
            self.mvcam = serial.Serial(
                port=comport,
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.01
            )
            print(f"Info: OpenMV is connected to {comport}")

        except serial.SerialException:
            print("Error: Serial port is not available.")
            print("OpenMV is not connected.")


    def search_for_blocks(self, duration_ms, pose, steps):
        """ Search for blocks with AprilTags.
        The robotic arm will be moved from the current pose to pose (argument) for duration_ms.
        During the transition from the current pose to the desired pose, the robot will stop to search
        for blocks. The number of stops during the transition is given by the steps argument.
        Once it finds a block, it will calculate the pose of the block wrt Frame 0 using the
        current joint angles and the position of the block wrt to the camera frame. Then, it will update
        self.block_locations with the pose of the block wrt Frame 0.
        If a block is detected multiple times, self.block_locations will be updated with the latest location.
        :param duration_ms: time in milliseconds to travel from the current
        position to the desired position.
        :param pose: 1x5 array of the pose of the tooltip.
        The first three elements are the position of the tooltip in meters,
        and the last two elements are the wrist angles in radians.
        :param steps: the number of stops during the transition
        :return: None

        Example:
        robot = XArm(simulation_only=False)
        robot.connect_mvcam()
        robot.connect()

        duration_ms = 2000
        steps = 20

        robot.move_to_initial_pose(duration_ms)
        pose = (0.12, -0.15, 0.12, -pi/4, 0)
        robot.search_for_blocks(duration_ms, pose, steps)
        """

        joint_angles = self.invkine(pose)

        if len(joint_angles) == 0:
            print("No solution to inverse kinematics for ", pose)
            return

        angle_step = (np.array(joint_angles[0]) - np.array(self.curr_joint_angles))/steps
        dtime_ms = duration_ms//steps

        # For each step, move the arm and search for AprilTags
        for i in range(steps):
            self.curr_joint_angles += angle_step
            self.move_joints(dtime_ms, self.curr_joint_angles, wait=True)

            # Transmit #H% to OpenMV to request any AprilTag detections
            self.mvcam.write(b'#H%')

            # Receive data from OpenMV
            data = self.mvcam.readline().decode('ascii').strip().split(',')

            # No detection. Skip the rest.
            if len(data) <= 1:
                continue
            # Number of Apriltags detected.
            num_tags = int(data.pop(0))

            # list to store Tag objects
            tags = list()
            posetag = list()
            robotpose1 = list()
            robotpose2 = list()
            # Create a Tag object for each Apriltag detected and
            # append it to the tags list.
            for i in range(num_tags):

                # Read tag ID.
                tagid = int(data[4*i])

                # Read the position of the tag wrt the camera frame
                cam_x, cam_y, cam_z = map(float, data[4*i+1:4+4*i])
                self.block_locations[tagid] = Tag(tagid, 0, 0, 0, cam_x, cam_y, cam_z)
                cpa = SE3(cam_x, cam_y, cam_z - self.camera_dist_offset)
                bpa = self.fkine(self.curr_joint_angles) * cpa
                # Ignore the AprilTag position wrt the base frame -> set it (0,0,0) for now.

                tags.append(Tag(tagid, bpa.t[0], bpa.t[1], bpa.t[2], cam_x, cam_y, cam_z))   #
                # for avg in tags:
                #     xsum = 0
                #     ysum = 0
                #     zsum = 0
                #
                #     avg.t[0] = avg.t[0] + x
                #
                #     avg.t[1] = avg.t[1] + y
                #
                #     avg.t[2] = avg.t[2] + z
                #
                #     xavg = xsum / len(avg.t[0])
                #     yavg = ysum / len(avg.t[1])
                #     zavg = zsum / len(avg.t[2])


                posetag.append(Pose(tagid, bpa.t[0], bpa.t[1], bpa.t[2], -pi/4, 0))
                sum_x = [0, 0, 0, 0, 0]
                sum_y = [0, 0, 0, 0, 0]
                sum_z = [0, 0, 0, 0, 0]
                for tag in tags:

                    len(data) == 0
                    print(['%0.4f'% t for t in tag])
        for p in posetag:

            sum_x[p.id] = sum_x[p.id] + p.x
            sum_y[p.id] = sum_y[p.id] + p.y
            sum_z[p.id] = sum_z[p.id]+ p.z
        avg_x = np.array(sum_x) / len(posetag)
        avg_y = np.array(sum_y) / len(posetag)
        avg_z = np.array(sum_z) / len(posetag)

        robotpose1 = [avg_x[0], avg_y[0], avg_z[0], -pi/3, 0]
        robotpose2 = [avg_x[1], avg_y[1], avg_z[1], -pi/4, 0]

            #posetag.append(Pose(tagid, avg_x, avg_y, avg_z, -pi/4 , 0))
            # Print the AprilTags detected by the OpemMV camera.
        self.open_gripper(2000)
        self.moveto(3000, (0.2225, -0.0184, -0.0098, -pi/3, 0) )
        #self.moveto(3000, robotpose1[0], robotpose1[1], robotpose1[2] + 0.03, robotpose1[3], robotpose1[4])
        self.close_gripper(2000)



    @property
    def simulation_only(self):
        return self._simulation_only

    @simulation_only.setter
    def simulation_only(self, value):
        self._simulation_only = value

    def is_serial_open(self):
        if self.comm is None:
            return False
        else:
            return self.comm.isOpen()

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


    def joint_to_servo_angles(self, joint_angles, joint_names=None):
        """ Returns servo angles corresponding to robot joint angles.

                            /
                           /
                          /  theta
                    _____/_)___
                   |  Base    |
                   -----------
        shoulder angle, theta2 = 0 deg   ---> servo angle = -90 deg
        shoulder angle, theta2 = 90 deg  ---> servo angle = 0 deg
        shoulder angle, theta2 = 180 deg ---> servo angle = 90 deg

        :param joint_angles: a vector of no greater than 5 robot angles in radians.
        :param joint_names: a vector of joint names in the set ('base', 'shoulder', 'elbow', 'wpitch', 'wroll').
                The joint names inside joints do not have to be the same order in the above set.
        :return: an array of servo angles in radians.
        """

        if joint_names is None or joint_names == 'all':
            joint_names = ('base', 'shoulder', 'elbow', 'wpitch', 'wroll')

        # checking errors
        # If there are any errors, return None
        if joint_angles is None:
            print(f"Error: The number of joint angles can't be less than 1.")
            return  # return None

        if len(joint_angles) > 5:
            print(f"Error: The number of robot angles can't be greater than 5, but {len(joint_angles)} robot angles are provided.")
            return  # return None

        if len(joint_angles) != len(joint_names):
            print(f"Error: The number of joint angles, {len(joint_angles)} is not the same as that of joint names, {len(joint_names)}.")
            return  # return None

        for joint in joint_names:
            if joint not in self.ordered_joints:
                print(f"Error: Unidentified joint name, {joint}, is provided.")
                return  # return None
        # end of checking errors

        servo_angles = []

        for item, direction, offset in zip(self.ordered_joints, self.joint_servo_direction, self.joint_servo_offset):
            # 1. If a joint name inside joint_names exists in ordered_joints,
            #    we find the index of the joint name from joint_names.
            # 2. We find the associated joint angle using the index.
            # 3. Convert the joint angle to the corresponding servo angle
            if item in joint_names:
                angle = joint_angles[joint_names.index(item)]
                servo_angles.append(angle*direction + offset)

        return np.array(servo_angles)

    def servo_angles_to_positions(self, servo_angles, joint_names=None):
        """ Returns servo positions corresponding to servo angles.
                                    ---------
                       alpha ----->|  F(x)  |-----> servo position
           (-90 deg to +90 deg)    ---------         (500 - 2500)

        :param servo_angles: a vector of no greater than 5 servo angles in radians.
        :param joint_names: a vector of joint names in the set ('base', 'shoulder', 'elbow', 'wpitch', 'wroll').
                The joint names inside joints do not have to be the same order in the above set.
        :return: an array of servo positions in a range of (500, 2500).
        """

        if joint_names is None or joint_names == 'all':
            joint_names = ('base', 'shoulder', 'elbow', 'wpitch', 'wroll')

        # checking errors
        # If there are any errors, return None
        if servo_angles is None:
            print(f"Error: The number of servo angles can't be less than 1.")
            return  # return None

        if len(servo_angles) > 5:
            print(f"Error: The number of servo angles can't be greater than 5, but {len(servo_angles)} servo angles are provided")
            return  # return None

        if len(servo_angles) != len(joint_names):
            print(f"Error: The number of servo angles, {len(servo_angles)} is not the same as that of joint names, {len(joint_names)}.")
            return  # return None

        for joint in joint_names:
            if joint not in self.ordered_joints:
                print(f"Unidentified joint name, {joint}, is provided.")
                return  # return None
        # end of checking errors

        servo_positions = []

        for item in self.ordered_joints:
            # 1. If a joint name inside joint_names exists in ordered_joints,
            #    we find the index of the joint name from joint_names.
            # 2. We find the associated servo angle using the index.
            # 3. Convert the servo angle to the corresponding servo position
            if item in joint_names:
                angle = servo_angles[joint_names.index(item)]
                servo_positions.append(1500 + angle * 1000 / (pi / 2))

        return np.array(servo_positions).astype(int)

    def get_servo_command(self, duration_ms, servo_positions, joint_names=None):
        """ Returns servo command for all 5 joints except the gripper.
        :param duration_ms: the time in ms to travel from the current position to the desired position.
        :param servo_positions: a vector of no greater than 5 servo angles. The range must be between 500 and 2500
        :param joint_names: a vector of joint names in the set ('base', 'shoulder', 'elbow', 'wpitch', 'wroll').
                The joint names inside joints do not have to be the same order in the above set.
        :return: byte array to send to the robot.

        The byte array uses the following serial communication protocol.

        Serial Comm Protocol
        |  Header   | Data Length | Command | Parameters |
        | 0x55 0x55 | byte length |   CMD   | P1 ... PN  |

        byte length = Param Length + CMD + byte length itself = param length + 2
        CMD: CMD_SERVO_MOVE = 3 (Always 3 for ECE 387)
        Parameters:
         P1: The number of servos to control
         P2: Lower 8 bits of duration in ms
         P3: Higher 8 bits of duration in ms
         P4: Servo ID
         P5: Lower 8 bits of the servo position
         P6: Higher 8 bits of the servo position
         P7: Servo ID
         P8: Lower 8 bits of the servo position
         P9: Higher 8 bits of the servo position
                :
                :

        Example to move Servo 3 to 2000 (07D0h) (45 deg) in 4000 (0FA0h) ms
        packet = bytearray()
        packet.append(0x55)
        packet.append(0x55)
        packet.append(0x08)
        packet.append(0x03)
        packet.append(0x01)
        packet.append(0xA0)
        packet.append(0x0F)
        packet.append(0x03)
        packet.append(0xD0)
        packet.append(0x07)
        """

        if joint_names is None or joint_names == 'all':
            joint_names = ('base', 'shoulder', 'elbow', 'wpitch', 'wroll')

        # checking errors
        # If there are any errors, return None
        if servo_positions is None:
            print(f"Error: The number of servo positions can't be less than 1.")
            return  # return None

        if len(servo_positions) > 5:
            print(f"Error: The number of servo positions can't be greater than 5, but {len(servo_positions)} servo positions are provided.")
            return  # return None

        if len(servo_positions) != len(joint_names):
            print(f"Error: The number of servo angles, {len(servo_positions)} is not the same as that of joint names, {len(joint_names)}.")
            return  # return None

        for joint in joint_names:
            if joint not in self.ordered_joints:
                print(f"Unidentified joint name, {joint}, is provided.")
                return  # return None
        # end of checking errors

        packet = bytearray(XArm.HEADER)
        num_bytes = 5 + 3 * len(joint_names)  # byte length + CMD + num_servos + time duration (2) + servo positions
        packet.append(num_bytes)
        packet.append(XArm.COMMAND)
        packet.append(len(joint_names))
        packet.append(duration_ms & 0xff)
        packet.append((duration_ms >> 8) & 0xff)

        for i, item in enumerate(self.ordered_joints):
            # 1. If a joint name inside joints exists in the ordered joints,
            #    we find the index of the joint name from the joints array.
            # 2. We find the associated robot angle using the index.
            # 3. Convert the robot angle to the corresponding servo angle
            if item in joint_names:
                position = servo_positions[joint_names.index(item)]
                if position < 500 or position > 2500:
                    print(f"Error: A servo {i+1} position must be between 500 and 2500, but the provided position is {position}.")
                    return  # return None
                else:
                    packet.append(i + 1)
                    packet.append(position & 0xff)
                    packet.append((position >> 8) & 0xff)

        return packet

    def send_command(self, command):
        """ Send servo command to the robot
        :param command: servo command generated by get_servo_command().
        :return: None
        """

        if not command:
            logging.warning("Command is empty.")
        elif self._simulation_only:
            logging.info("The simulation-only mode is enabled. Command was not sent to the robot.")
        elif self.comm is None:
            logging.warning("Serial port is not available and the command was not sent to the robot.")
        else:
            self.comm.write(command)

    def move_joints(self, duration_ms, joint_angles, joint_names=None, wait=True):
        """ Moves the joints of the robot to the joint angles specified.
        :param duration_ms: time in milliseconds to travel from the current
        position to the desired position.
        :param joint_angles: desired joint angles to move.
        :param joint_names: names of joints angles to move.
        :param wait: If true, sleep for duration_ms until the move is complete.
        :return:
        """

        servo_angles = self.joint_to_servo_angles(joint_angles, joint_names)
        servo_positions = self.servo_angles_to_positions(servo_angles, joint_names)
        servo_command = self.get_servo_command(duration_ms, servo_positions, joint_names)
        self.send_command(servo_command)
        if wait and not self.simulation_only:
            sleep(duration_ms/1000)

    def move_to_initial_pose(self, duration_ms, wait=True):
        """ Sets the initial joint angles of the robot.
        The main purposes are (i) to define the reset position
        when the program starts and (ii) to set the current joint angles,
        which is used for the numerical approach of inverse kinematics.
        Note: duration_ms is not effective at the very beginning.
        :param duration_ms: time in milliseconds to travel from the current pose to the initial pose.
        :param wait: If true, sleep for duration_ms until the move is complete.
        :return: None
        """

        joint_angles = [0, pi/2, 0, 0, 0]
        self.move_joints(duration_ms, joint_angles)
        if wait and not self.simulation_only:
            sleep(duration_ms/1000)

        self.curr_joint_angles = np.array([0, pi/2, 0, 0, 0])



    def check_joint_angle_limits(self, angles):
        """ Check if angles are in the operating ranges of the joint angles.
        :param angles: angles to be checked
        :return: True if all angles are in the operating ranges of the joint angles.
                False if any of them is not in the operating ranges of the joint angles.
        """

        for angle, max_angle, min_angle in zip(angles, self.max_joint_angle, self.min_joint_angle):
            if min_angle <= angle <= max_angle:
                pass
            else:
                return False

        return True

    def invkine(self, pose):
        """ Returns the closed-form solutions to inverse kinematics.
        :param: pose: 1x5 array for the pose of the tooltip.
        The first three elements are the position of the tooltip in meters,
        and the last two elements are the wrist angles in radians.
        :return: an array of joint angles in radians.
        The array is empty if there exists no solution,
        a 1x5 array if there exists a unique solution, and
        an Nx5 array if there exist N solutions.

        The joint angles returned by this function are bounded by
        self.max_joint_angle and self.min_joint_angle.

        Example:
        pose = [px, py, pz, phi, psi]
        robot = XArm()
        thetas = robot.invkine(pose)

        If no solution exists, thetas = []
        If one solution exists, thetas = [[theta1, theta2, theta3, theta4, theta5]]
        If two solution exist, thetas = [[theta1, theta2, theta3, theta4, theta5],
                                         [theta1, theta2, theta3, theta4, theta5]]
        """

        px = pose[0]
        py = pose[1]
        pz = pose[2]
        phi = pose[3]
        psi = pose[4]
        theta5 = psi
        # Write your code here for Project 1

        # Note:
        # The joint angles returned by this function must be bounded by
        # self.max_joint_angle and self.min_joint_angle
        theta1 = np.arctan2(py, px)
        x = px - self.L4 * np.cos(phi) * np.cos(theta1)
        y = py - self.L4 * np.cos(phi) * np.sin(theta1)
        z = pz - self.L4 * np.sin(phi) - self.B0

        r = np.sqrt(x**2 + y**2) - self.L1
        d = np.sqrt(r**2 + z**2)
        beta = np.arctan2(z, r)
        theta3 = np.arccos((d**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3))
        alpha = np.arccos((d**2 + self.L2**2 - self.L3**2) / (2 * self.L2 * d))
        #theta4 = phi - theta2 - theta3
        if d > self.L2 + self.L3:
            return []
        elif np.isclose(d, self.L2 + self.L3):
            #theta2 = beta - (theta3 * alpha)
            theta4 = phi - beta - theta3
            joint_angles2 = [theta1, beta, 0, theta4, theta5]
            if self.check_joint_angle_limits(joint_angles2):
                return [joint_angles2]
            else:
                return []

        elif np.isclose(d, 0):
            theta4 = phi - 0 - theta3
            jangles = [[theta1, 0, theta3, theta4, theta5], [theta1, 0, -theta3, theta4, theta5]]
            key1 = []
            for j in jangles:
                if self.check_joint_angle_limits(j):
                    key1.append(j)
            return key1

        elif d < self.L2 + self.L3:
            theta2one = beta - alpha
            theta2two = beta + alpha
            theta4one = phi - theta2one - theta3
            theta4two = phi - theta2two + theta3
            #theta2 = beta - (theta3 * alpha)
            angles1 = [[theta1, theta2one, theta3, theta4one, theta5], [theta1, theta2two, -theta3, theta4two, theta5]]
            key = []
            for ang in angles1:
                if self.check_joint_angle_limits(ang):
                    key.append(ang)
            return key

    def moveto(self, duration_ms, pose, wait=True, plot=False):
        """ Moves the tooltip to the specified pose.
        :param duration_ms: time in milliseconds to travel from the current
        position to the desired position.
        :param pose: 1x5 array of the pose of the tooltip.
        The first three elements are the position of the tooltip,
        and the last two elements are the wrist angles.
        :param wait: If true, sleep for duration_ms until the move is complete.
        :param plot: If true, plot the robot.
        :return: None
        """

        joint_angles = self.invkine(pose)

        if joint_angles is None:
            print("No solution to inverse kinematics for ", pose)
        else:
            self.move_joints(duration_ms, joint_angles[0], wait=wait)
            if plot:
                self.plot(joint_angles, limits=self.plot_axes_limits, block=True)

            self.curr_joint_angles = joint_angles[0]

    def open_gripper(self, duration_ms, wait=True):
        """ Open gripper.
        :param duration_ms: time in milliseconds to open the gripper.
        :param wait: If true, sleep for duration_ms until the move is complete.
        :return: None
        The close position is set in self.gripper_open_pos
        """
        self.move_gripper(duration_ms, self.gripper_open_pos, wait)

    def close_gripper(self, duration_ms, wait=True):
        """ Close gripper.
        :param duration_ms: time in milliseconds to close the gripper.
        :param wait: If true, sleep for duration_ms until the move is complete.
        :return: None
        The close position is set in self.gripper_close_pos
        """
        self.move_gripper(duration_ms, self.gripper_close_pos, wait)

    def move_gripper(self, duration_ms, position, wait=True):
        """ Open or close gripper.
        This is a helper function for open_gripper and close_gripper
        :param duration_ms: time in milliseconds to move the gripper.
        :param position: servo position in microseconds in the range of [500, 2500]
        :param wait: If true, sleep for duration_ms until the move is complete.
        :return: None
        """

        packet = bytearray(XArm.HEADER)
        num_bytes = 5 + 3  # byte length + CMD + num_servos + time duration (2) + servo positions
        packet.append(num_bytes)
        packet.append(XArm.COMMAND)
        packet.append(1)    # only one servo
        packet.append(duration_ms & 0xff)
        packet.append((duration_ms >> 8) & 0xff)
        packet.append(6)    # servo 6: gripper
        packet.append(position & 0xff)
        packet.append((position >> 8) & 0xff)

        self.send_command(packet)
        if wait:
            sleep(duration_ms/1000)




""" ======================================================================
    End of Class XArm
    ====================================================================== """


def test_forward_kinematics():
    robot = XArm(simulation_only=False)
    duration_ms = 2000
    robot.connect()
    robot.move_to_initial_pose(duration_ms, wait=True)

    q = list()
    q.append([0, pi/2, 0, 0, 0])  # rest position
    q.append([0, 0, 0, 0, 0])     # zero position
    q.append([0, pi/2, -pi/2, 0, 0])
    q.append([0, pi/2, pi/2, 0, 0])
    q.append([0, pi/2, 0, -pi/2, 0])
    q.append([0, pi/2, 0, pi/2, 0])
    q.append([0, pi/2, 0, 0, -pi/2])
    q.append([0, pi/2, 0, 0, pi/2])
    q.append([-pi/2, pi/2, -pi/2, 0, -pi/2])
    q.append([pi/2, pi/2, pi/2, 0, pi/2])

    for angles in q:
        """
        Write your code to find the forward kinematics and generate plots.
        Hints:
        - robot is an instance of XArm, which is a subclass of DHRobot. So you can use fkine.
        - use printline to print pose and angle. In Python Console, try
             from spatialmath import SE3
             help(SE3.printline)
          to get help on function printline.
          
        Angles must be reported in degrees, not radians.
        """

        T = robot.fkine(angles)
        # print(T)
        print(np.array(angles)* (180/pi))
        robot.move_joints(duration_ms, angles, wait=True)
        T.printline()





def test_servo_angle_command():

    robot = XArm(simulation_only=False)
    robot.connect()

    duration_ms = 2000

    # initial rest position
    servo_positions = robot.servo_angles_to_positions((0, 0, pi/4, 0, 0))
    servo_command = robot.get_servo_command(duration_ms, servo_positions)
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # base = pi/4 and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((pi/4,), ('base',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('base',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # base = -pi/4 and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((-pi/4,), ('base',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('base',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # base = 0, shoulder = pi/4, and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((0, pi/4,), ('base', 'shoulder',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('base', 'shoulder',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # shoulder = -pi/4, and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((-pi/4,), ('shoulder',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('shoulder',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # shoulder = 0, elbow = 0,and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((0, 0,), ('shoulder', 'elbow',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('shoulder', 'elbow',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # elbow = pi/2 and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((pi/2,), ('elbow',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('elbow',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # elbow = pi/4, wpitch = -pi/4, and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((pi/4, -pi/4,), ('elbow', 'wpitch',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('elbow', 'wpitch'))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # wpitch = pi/4 and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((pi/4,), ('wpitch',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('wpitch',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # wpitch = 0, wroll = -pi/4,and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((0, -pi/4,), ('wpitch', 'wroll',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('wpitch', 'wroll',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()

    # wroll = pi/4 and keep the rest as it is
    servo_positions = robot.servo_angles_to_positions((pi/4,), ('wroll',))
    servo_command = robot.get_servo_command(duration_ms, servo_positions, ('wroll',))
    robot.send_command(servo_command)
    sleep(duration_ms/1000)
    breakpoint()


def test_move_joints():

    robot = XArm(simulation_only=False)
    robot.connect()

    duration_ms = 2000
    robot.move_to_initial_pose(duration_ms, wait=True)

    # base = pi/4 and keep the rest as it is
    robot.move_joints(duration_ms, (pi/4,), ('base',), wait=True)

    # base = -pi/4 and keep the rest as it is
    robot.move_joints(duration_ms, (-pi/4,), ('base',), wait=True)

    # base = 0, shoulder = pi/4, and keep the rest as it is
    robot.move_joints(duration_ms, (0, pi/4,), ('base', 'shoulder',), wait=True)

    # shoulder = 45 and keep the rest as it is
    robot.move_joints(duration_ms, (pi/4, ), ('shoulder',), wait=True)

    # shoulder = 135 and keep the rest as it is
    robot.move_joints(duration_ms, (3*pi/4, ), ('shoulder',), wait=True)
    # shoulder = 90, elbow = 45,and keep the rest as it is
    robot.move_joints(duration_ms, (pi/2, pi/4, ), ('shoulder', 'elbow',), wait=True)
    # elbow = -pi/2 and keep the rest as it is
    robot.move_joints(duration_ms, (-pi/2, ), ('elbow',), wait=True)

    # elbow = 0, wpitch = -pi/4, and keep the rest as it is
    robot.move_joints(duration_ms, (0, -pi/4, ), ('elbow', 'wpitch',), wait=True)

    # wpitch = pi/4 and keep the rest as it is
    robot.move_joints(duration_ms, (pi/4, ), ('wpitch',), wait=True)

    # wpitch = 0, wroll = -pi/4,and keep the rest as it is
    robot.move_joints(duration_ms, (0, -pi/4, ), ('wpitch', 'wroll',), wait=True)

    # wroll = pi/4 and keep the rest as it is
    robot.move_joints(duration_ms, (pi/4, ), ('wroll',), wait=True)


def test_inverse_kinematics():

    robot = XArm(simulation_only=False)
    robot.connect()
    duration_ms = 2000
    robot.move_to_initial_pose(duration_ms)

    poses = list()
    """
    poses.append([robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0])
    poses.append([robot.L1 + robot.L2 + robot.L3, 0, robot.L4 + robot.B0, pi/2, 0])
    poses.append([robot.L1 + robot.L2 + robot.L3 + robot.L4, 0, robot.B0, 0, 0])
    poses.append([robot.L1 + robot.L2 + robot.L3 + robot.L4, 0, robot.B0, 0, pi/4])
    poses.append([(robot.L1 + robot.L3 + robot.L4) * np.cos(np.deg2rad(45)),
                  (robot.L1 + robot.L3 + robot.L4) * np.sin(np.deg2rad(45)),
                  robot.L2 + robot.B0, 0, 0])
    poses.append([(robot.L1 + robot.L3 + robot.L4) * np.cos(np.deg2rad(-45)),
                  (robot.L1 + robot.L3 + robot.L4) * np.sin(np.deg2rad(-45)),
                  robot.L2 + robot.B0, 0, pi/4])
    poses.append([(robot.L1 + robot.L3 + robot.L4) * np.cos(np.deg2rad(-45)),
                  (robot.L1 + robot.L3 + robot.L4) * np.sin(np.deg2rad(-45)),
                  robot.L2 + robot.B0, 0, -pi/4])
    poses.append([(robot.L1 + robot.L3 + robot.L4) * np.cos(np.deg2rad(60)),
                  (robot.L1 + robot.L3 + robot.L4) * np.sin(np.deg2rad(60)),
                  robot.L2 + robot.B0, 0, 0])
    poses.append([0, robot.L1 + robot.L2 + robot.L3 + robot.L4, robot.B0, 0, 0])
    poses.append([0, 0, robot.B0, -pi/2, 0])
    poses.append([0.16, 0.16,  0.23, 0, 0])
    poses.append([0.16, -0.16,  0.23, 0, 0])
    poses.append([0, 0.16,  0.27, 0, 0])
    poses.append([0, -0.16,  0.27, 0, 0])
    poses.append([0, 0,  0.45, pi/2, 0])
    poses.append([0, 0,  0.35, pi/2, 0])
    """

    # Deliverable 3
    robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.12, -pi/6, 0))
    #poses.append([0.24, -0.055,  0, -pi/4, 0])
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.17, -pi/6, 0))
    robot.moveto(2000, (0.19, 0.15,  0, -pi/4, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))

    #robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.09, -pi/6, 0))
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.17, -pi/6, 0))
    robot.moveto(2000, (0.19, 0.15,  0.03, -pi/4, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))

    #robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.06, -pi/6, 0))
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.17, -pi/6, 0))
    robot.moveto(2000, (0.19, 0.15,  0.075, -pi/4, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))

    #robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.03, -pi/6, 0))
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.17, -pi/6, 0))
    robot.moveto(2000, (0.19, 0.15,  0.11, -pi/4, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))

    #robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0, -pi/4, 0))
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.17, -pi/6, 0))
    robot.moveto(2000, (0.19, 0.15,  0.145, -pi/6, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))




    '''
    #Deliverable 2
    robot.open_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0, -pi/4, 0))
    #poses.append([0.24, -0.055,  0, -pi/4, 0])
    robot.close_gripper(2000)
    robot.moveto(2000, (0.24, -0.055,  0.1, -pi/4, 0))
    robot.moveto(2000, (0.19, 0.15,  0, -pi/4, 0))
    robot.open_gripper(2000)
    robot.moveto(2000, (robot.L1, 0, robot.L2 + robot.L3 + robot.L4 + robot.B0, pi/2, 0))
    '''

    #poses.append([0.20, -0.16,  0, 0, 0])


    poses = np.array(poses)
    axes_limits = [-0.38, 0.38, -0.38, 0.38, 0, 0.47]
    np.set_printoptions(precision=3, suppress=True)

    for pose in poses:
        print("Desired pose: ", pose[0:3], pose[3:] * 180 / pi)
        joint_angles = robot.invkine(pose)

        if joint_angles is not None:
            print(np.rad2deg(joint_angles))

            for angles in joint_angles:
                T = robot.fkine(angles)
                T.printline(orient='eul')
                robot.move_joints(duration_ms, angles)
                robot.plot(angles, limits=axes_limits, block=True)

    robot.disconnect()

def test_openmv():

        np.set_printoptions(precision=3, suppress=True)

        robot = XArm(simulation_only=True)
        robot.connect_mvcam()
        robot.mvcam.flushInput()

        while True:

            sleep(0.1)

            # Transmit #H% to OpenMV to request any AprilTag detections
            robot.mvcam.write(b'#H%')

            # Receive data from OpenMV
            data = robot.mvcam.readline().decode('ascii').strip().split(',')

            # No detection. Skip the rest.
            if len(data) <= 1:
                continue

            # Number of Apriltags detected.
            num_tags = int(data.pop(0))

            # list to store Tag objects
            tags = list()

            # Create a Tag object for each Apriltag detected and
            # append it to the tags list.
            for i in range(num_tags):

                # Read tag ID.
                tagid = int(data[4*i])

                # Read the position of the tag wrt the camera frame
                cam_x, cam_y, cam_z = map(float, data[4*i+1:4+4*i])

                # Ignore the AprilTag position wrt the base frame -> set it (0,0,0) for now.
                tags.append(Tag(tagid, 0, 0, 0, cam_x, cam_y, cam_z))   #

            # Print the AprilTags detected by the OpemMV camera.
            for tag in tags:
                print(['%0.4f'% t for t in tag])

def test_search_for_blocks():

    # Set simulation_only=True to test the OpenMV cam.
    # Set it to False to test the camera while the robot is moving.

    robot = XArm(simulation_only=False)
    robot.connect_mvcam()
    robot.connect()

    duration_ms = 7000
    steps = 50  # pick a number for steps.

    robot.move_to_initial_pose(duration_ms)


    pose = (0.03, -0.18,  0.05, -pi/4, 0)  # pick the start pose for searching
    robot.moveto(duration_ms, pose, wait=True)        # Transmit #H% to OpenMV to request any AprilTag detections

    pose = (0.03, 0.15,  0.05, -pi/4, 0)  # pick the destination pose for searching
    robot.search_for_blocks(duration_ms, pose, steps)

    pose = (0.10, -0.15,  0.08, -pi/6, 0) # pick the next destination pose for searching
    robot.search_for_blocks(duration_ms, pose, steps)

    pose = (0.10, -0.15,  0.08, -pi/6, 0) # pick the next destination pose for searching
    robot.search_for_blocks(duration_ms, pose, steps)

if __name__ == '__main__':

    logging.basicConfig(level=logging.INFO)

    # test_forward_kinematics()
    # test_servo_position_command()
    # test_servo_angle_command()
    # test_move_joints()
    # test_gripper()
    #test_inverse_kinematics()
    # test_search_for_blocks()
    #test_openmv()
    test_search_for_blocks()
