

    def check_joint_angle_limits(self, angles):
        """ Check if angles are in the operating ranges of the joint angles.
        :param angles: angles to be checked
        :return: True if all angles are in the operating ranges of the joint angles.
                False if any of them is not in the operating ranges of the joint angles.
        """

        for angle, max_angle, min_angle in zip(angles, self.max_joint_angle, self.min_joint_angle):
            if min_angle < angle < max_angle:
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

        # Write your code here for Project 1
        
        # Note:
        # The joint angles returned by this function must be bounded by
        # self.max_joint_angle and self.min_joint_angle


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
            self.move_joints(duration_ms, joint_angles[0, :], wait=wait)
            if plot:
                self.plot(joint_angles, limits=self.plot_axes_limits, block=True)

            self.curr_joint_angles = joint_angles[0, :]

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




def test_inverse_kinematics():

    robot = XArm(simulation_only=True)
    robot.connect()
    duration_ms = 2000
    robot.move_to_initial_pose(duration_ms)

    poses = list()

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
    poses.append([0, -(robot.L1 + robot.L2 + robot.L3 + robot.L4), robot.B0, 0, 0])
    poses.append([0, 0, robot.B0, -pi/2, 0])
    poses.append([0.16, 0.16,  0.23, 0, 0])
    poses.append([0.16, -0.16,  0.23, 0, 0])
    poses.append([0, 0.16,  0.27, 0, 0])
    poses.append([0, -0.16,  0.27, 0, 0])
    poses.append([0, 0,  0.45, pi/2, 0])
    poses.append([0, 0,  0.35, pi/2, 0])

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

