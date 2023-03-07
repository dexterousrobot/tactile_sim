import sys
import numpy as np


class BaseRobotArm:
    def __init__(
        self,
        pb,
        embodiment_id,
        workframe,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses,
        tcp_lims
    ):

        self._pb = pb
        self.rest_poses = rest_poses  # default joint pose for ur5
        self.embodiment_id = embodiment_id
        self.num_joints = self._pb.getNumJoints(embodiment_id)
        self.tcp_link_id = tcp_link_id
        self.link_name_to_index = link_name_to_index
        self.joint_name_to_index = joint_name_to_index

        # set up the work frame
        self.set_workframe(workframe)
        self.set_tcp_lims(tcp_lims)

    def reset(self):
        """
        Reset the UR5 to its rest positions and hold.
        """
        # reset the joint positions to a rest position
        for i in range(self.num_joints):
            self._pb.resetJointState(self.embodiment_id, i, self.rest_poses[i])
            self._pb.changeDynamics(self.embodiment_id, i, linearDamping=0.04, angularDamping=0.04)
            self._pb.changeDynamics(self.embodiment_id, i, jointDamping=0.01)

        # hold in rest pose
        self._pb.setJointMotorControlArray(
            bodyIndex=self.embodiment_id,
            jointIndices=self.control_joint_ids,
            controlMode=self._pb.POSITION_CONTROL,
            targetPositions=self.rest_poses[self.control_joint_ids],
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=np.zeros(self.num_control_dofs) + self.max_force,
        )

    def set_workframe(self, frame):
        """
        set the working coordinate frame (should be expressed relative to world frame)
        """
        self.workframe_pos = np.array(frame[:3])
        self.workframe_rpy = np.array(frame[3:])
        self.workframe_orn = np.array(self._pb.getQuaternionFromEuler(self.workframe_rpy))

    def workframe_to_worldframe(self, pos, rpy):
        """
        Transforms a pose in work frame to a pose in world frame.
        """

        pos = np.array(pos)
        rpy = np.array(rpy)
        orn = np.array(self._pb.getQuaternionFromEuler(rpy))

        worldframe_pos, worldframe_orn = self._pb.multiplyTransforms(self.workframe_pos, self.workframe_orn, pos, orn)
        worldframe_rpy = self._pb.getEulerFromQuaternion(worldframe_orn)

        return np.array(worldframe_pos), np.array(worldframe_rpy)

    def worldframe_to_workframe(self, pos, rpy):
        """
        Transforms a pose in world frame to a pose in work frame.
        """
        pos = np.array(pos)
        rpy = np.array(rpy)
        orn = np.array(self._pb.getQuaternionFromEuler(rpy))

        inv_workframe_pos, inv_workframe_orn = self._pb.invertTransform(self.workframe_pos, self.workframe_orn)
        workframe_pos, workframe_orn = self._pb.multiplyTransforms(inv_workframe_pos, inv_workframe_orn, pos, orn)
        workframe_rpy = self._pb.getEulerFromQuaternion(workframe_orn)

        return np.array(workframe_pos), np.array(workframe_rpy)

    def workvec_to_worldvec(self, workframe_vec):
        """
        Transforms a vector in work frame to a vector in world frame.
        """
        workframe_vec = np.array(workframe_vec)
        rot_matrix = np.array(self._pb.getMatrixFromQuaternion(self.workframe_orn)).reshape(3, 3)
        worldframe_vec = rot_matrix.dot(workframe_vec)

        return np.array(worldframe_vec)

    def worldvec_to_workvec(self, worldframe_vec):
        """
        Transforms a vector in world frame to a vector in work frame.
        """
        worldframe_vec = np.array(worldframe_vec)
        inv_workframe_pos, inv_workframe_orn = self._pb.invertTransform(self.workframe_pos, self.workframe_orn)
        rot_matrix = np.array(self._pb.getMatrixFromQuaternion(inv_workframe_orn)).reshape(3, 3)
        workframe_vec = rot_matrix.dot(worldframe_vec)

        return np.array(workframe_vec)

    def workvel_to_worldvel(self, workframe_pos_vel, workframe_ang_vel):
        """
        Convert linear and angular velocities in workframe to worldframe.
        """
        rot_matrix = np.array(self._pb.getMatrixFromQuaternion(self.workframe_orn)).reshape(3, 3)

        worldframe_pos_vel = rot_matrix.dot(workframe_pos_vel)
        worldframe_ang_vel = rot_matrix.dot(workframe_ang_vel)

        return worldframe_pos_vel, worldframe_ang_vel

    def worldvel_to_workvel(self, worldframe_pos_vel, worldframe_ang_vel):
        """
        Convert linear and angular velocities in worldframe to workframe.
        """

        inv_workframe_pos, inv_workframe_orn = self._pb.invertTransform(self.workframe_pos, self.workframe_orn)
        rot_matrix = np.array(self._pb.getMatrixFromQuaternion(inv_workframe_orn)).reshape(3, 3)

        workframe_pos_vel = rot_matrix.dot(worldframe_pos_vel)
        workframe_ang_vel = rot_matrix.dot(worldframe_ang_vel)

        return workframe_pos_vel, workframe_ang_vel

    def set_tcp_lims(self, lims):
        """
        Used to limit the range of the TCP
        """
        self.tcp_lims = lims

    def get_current_joint_pos_vel(self):
        """
        Get the current joint states of the ur5
        """
        cur_joint_states = self._pb.getJointStates(self.embodiment_id, self.control_joint_ids)
        cur_joint_pos = [cur_joint_states[i][0] for i in range(self.num_control_dofs)]
        cur_joint_vel = [cur_joint_states[i][1] for i in range(self.num_control_dofs)]

        return cur_joint_pos, cur_joint_vel

    def get_current_TCP_pos_vel_worldframe(self):
        """
        Get the current velocity of the TCP
        """
        tcp_state = self._pb.getLinkState(
            self.embodiment_id,
            self.tcp_link_id,
            computeLinkVelocity=True,
            computeForwardKinematics=False,
        )
        tcp_pos = np.array(tcp_state[0])  # worldLinkPos
        tcp_orn = np.array(tcp_state[1])  # worldLinkOrn
        tcp_rpy = self._pb.getEulerFromQuaternion(tcp_orn)
        tcp_lin_vel = np.array(tcp_state[6])  # worldLinkLinearVelocity
        tcp_ang_vel = np.array(tcp_state[7])  # worldLinkAngularVelocity
        return tcp_pos, tcp_rpy, tcp_orn, tcp_lin_vel, tcp_ang_vel

    def get_current_TCP_pos_vel_workframe(self):
        # get sim info on TCP
        (
            tcp_pos,
            tcp_rpy,
            tcp_orn,
            tcp_lin_vel,
            tcp_ang_vel,
        ) = self.get_current_TCP_pos_vel_worldframe()
        tcp_pos_workframe, tcp_rpy_workframe = self.worldframe_to_workframe(tcp_pos, tcp_rpy)
        tcp_orn_workframe = self._pb.getQuaternionFromEuler(tcp_rpy_workframe)
        tcp_lin_vel_workframe, tcp_ang_vel_workframe = self.worldvel_to_workvel(tcp_lin_vel, tcp_ang_vel)

        return (
            tcp_pos_workframe,
            tcp_rpy_workframe,
            tcp_orn_workframe,
            tcp_lin_vel_workframe,
            tcp_ang_vel_workframe,
        )

    def compute_gravity_compensation(self):
        cur_joint_pos, cur_joint_vel = self.get_current_joint_pos_vel()
        grav_comp_torque = self._pb.calculateInverseDynamics(
            self.embodiment_id, cur_joint_pos, cur_joint_vel, [0] * self.num_control_dofs
        )
        return np.array(grav_comp_torque)

    def apply_gravity_compensation(self):
        grav_comp_torque = self.compute_gravity_compensation()

        self._pb.setJointMotorControlArray(
            bodyIndex=self.embodiment_id,
            jointIndices=self.control_joint_ids,
            controlMode=self._pb.TORQUE_CONTROL,
            forces=grav_comp_torque,
        )

    def tcp_direct_workframe_move(self, target_pos, target_rpy):
        """
        Go directly to a position specified relative to the workframe
        """

        # transform from work_frame to world_frame
        target_pos, target_rpy = self.workframe_to_worldframe(target_pos, target_rpy)
        target_orn = np.array(self._pb.getQuaternionFromEuler(target_rpy))

        # get target joint poses through IK
        joint_poses = self._pb.calculateInverseKinematics(
            self.embodiment_id,
            self.tcp_link_id,
            target_pos,
            target_orn,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-8,
        )
        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_poses,
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self.target_pos_worldframe = target_pos
        self.target_rpy_worldframe = target_rpy
        self.target_orn_worldframe = target_orn
        self.target_joints = joint_poses

    def tcp_position_control(self, desired_delta_pose):
        """
        Actions specifiy desired changes in position in the workframe.
        TCP limits are imposed.
        """
        # get current position
        (
            cur_tcp_pos,
            cur_tcp_rpy,
            cur_tcp_orn,
            _,
            _,
        ) = self.get_current_TCP_pos_vel_workframe()

        # add actions to current positions
        target_pos = cur_tcp_pos + np.array(desired_delta_pose[:3])
        target_rpy = cur_tcp_rpy + np.array(desired_delta_pose[3:])

        # limit actions to safe ranges
        target_pos, target_rpy = self.check_TCP_pos_lims(target_pos, target_rpy)

        # convert to worldframe coords for IK
        target_pos, target_rpy = self.workframe_to_worldframe(target_pos, target_rpy)
        target_orn = self._pb.getQuaternionFromEuler(target_rpy)

        # get joint positions using inverse kinematics
        joint_poses = self._pb.calculateInverseKinematics(
            self.embodiment_id,
            self.tcp_link_id,
            target_pos,
            target_orn,
            restPoses=self.rest_poses,
            maxNumIterations=100,
            residualThreshold=1e-8,
        )
        # set joint control
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.POSITION_CONTROL,
            targetPositions=joint_poses,
            targetVelocities=[0] * self.num_control_dofs,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

        # set target positions for blocking move
        self.target_pos_worldframe = target_pos
        self.target_rpy_worldframe = target_rpy
        self.target_orn_worldframe = target_orn
        self.target_joints = joint_poses

    def tcp_velocity_control(self, desired_tcp_vels):
        """
        Actions specifiy desired velocities in the workframe.
        TCP limits are imposed.
        """
        # check that this won't push the TCP out of limits
        # zero any velocities that will
        capped_desired_vels = self.check_TCP_vel_lims(np.array(desired_tcp_vels))

        # convert desired vels from workframe to worldframe
        capped_desired_vels[:3], capped_desired_vels[3:] = self.workvel_to_worldvel(
            capped_desired_vels[:3], capped_desired_vels[3:]
        )

        # get current joint positions and velocities
        q, qd = self.get_current_joint_pos_vel()

        # calculate the jacobian for tcp link
        # used to map joing velocities to TCP velocities
        jac_t, jac_r = self._pb.calculateJacobian(
            self.embodiment_id,
            self.tcp_link_id,
            [0, 0, 0],
            q,
            qd,
            [0] * self.num_control_dofs,
        )

        # merge into one jacobian matrix
        jac = np.concatenate([np.array(jac_t), np.array(jac_r)])

        # invert the jacobian to map from tcp velocities to joint velocities
        # be careful of singnularities and non square matrices
        # use pseudo-inverse when this is the case
        # this is all the time for 7 dof arms like panda
        if jac.shape[1] > np.linalg.matrix_rank(jac.T):
            inv_jac = np.linalg.pinv(jac)
        else:
            inv_jac = np.linalg.inv(jac)

        # convert desired velocities from cart space to joint space
        req_joint_vels = np.matmul(inv_jac, capped_desired_vels)

        # apply joint space velocities
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.VELOCITY_CONTROL,
            targetVelocities=req_joint_vels,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

    def joint_velocity_control(self, desired_joint_vels):
        """
        Actions specify desired joint velicities.
        No Limits are imposed.
        """
        self._pb.setJointMotorControlArray(
            self.embodiment_id,
            self.control_joint_ids,
            self._pb.VELOCITY_CONTROL,
            targetVelocities=desired_joint_vels,
            positionGains=[self.pos_gain] * self.num_control_dofs,
            velocityGains=[self.vel_gain] * self.num_control_dofs,
            forces=[self.max_force] * self.num_control_dofs,
        )

    def step_sim(self):
        """
        Take a step of the simulation whilst applying neccessary forces
        """

        # compensate for the effect of gravity
        self.apply_gravity_compensation()

        # step the simulation
        self._pb.stepSimulation()

    def apply_action(
        self,
        motor_commands,
        control_mode="TCP_velocity_control",
        velocity_action_repeat=1,
        max_steps=100,
    ):

        if control_mode == "TCP_position_control":
            self.tcp_position_control(motor_commands)
        elif control_mode == "TCP_velocity_control":
            self.tcp_velocity_control(motor_commands)
        elif control_mode == "joint_velocity_control":
            self.joint_velocity_control(motor_commands)
        else:
            sys.exit("Incorrect control mode specified: {}".format(control_mode))

        if control_mode == "TCP_position_control":
            # repeatedly step the sim until a target pose is met or max iters
            self.blocking_move(max_steps=max_steps, constant_vel=None)

        elif control_mode in ["TCP_velocity_control", "joint_velocity_control"]:
            # apply the action for n steps to match control rate
            for i in range(velocity_action_repeat):
                self.step_sim()
        else:
            # just do one step of the sime
            self.step_sim()

    def blocking_move(
        self,
        max_steps=1000,
        constant_vel=None,
        pos_tol=2e-4,
        orn_tol=1e-3,
        jvel_tol=0.1,
    ):
        """
        step the simulation until a target position has been reached or the max
        number of steps has been reached
        """
        # get target position
        targ_pos = self.target_pos_worldframe
        targ_orn = self.target_orn_worldframe
        targ_j_pos = self.target_joints

        pos_error = 0.0
        orn_error = 0.0
        for i in range(max_steps):

            # get the current position and veloicities (worldframe)
            (
                cur_TCP_pos,
                cur_TCP_rpy,
                cur_TCP_orn,
                _,
                _,
            ) = self.get_current_TCP_pos_vel_worldframe()

            # get the current joint positions and velocities
            cur_j_pos, cur_j_vel = self.get_current_joint_pos_vel()

            # Move with constant velocity (from google-ravens)
            # break large position move to series of small position moves.
            if constant_vel is not None:
                diff_j = np.array(targ_j_pos) - np.array(cur_j_pos)
                norm = np.linalg.norm(diff_j)
                v = diff_j / norm if norm > 0 else np.zeros_like(cur_j_pos)
                step_j = cur_j_pos + v * constant_vel

                # reduce vel if joints are close enough,
                # this helps to acheive final pose
                if all(np.abs(diff_j) < constant_vel):
                    constant_vel /= 2

                # set joint control
                self._pb.setJointMotorControlArray(
                    self.embodiment_id,
                    self.control_joint_ids,
                    self._pb.POSITION_CONTROL,
                    targetPositions=step_j,
                    targetVelocities=[0.0] * self.num_control_dofs,
                    positionGains=[self.pos_gain] * self.num_control_dofs,
                    velocityGains=[self.vel_gain] * self.num_control_dofs
                )

            # step the simulation
            self.step_sim()

            # calc totoal velocity
            total_j_vel = np.sum(np.abs(cur_j_vel))

            # calculate the difference between target and actual pose
            pos_error = np.sum(np.abs(targ_pos - cur_TCP_pos))
            orn_error = np.arccos(
                np.clip((2 * (np.inner(targ_orn, cur_TCP_orn) ** 2)) - 1, -1, 1)
            )

            # break if the pose error is small enough
            # and the velocity is low enough
            if (pos_error < pos_tol) and (orn_error < orn_tol) and (total_j_vel < jvel_tol):
                break

    def check_TCP_pos_lims(self, pos, rpy):
        """
        cap the pos at the TCP limits specified
        """
        pos = np.clip(pos, self.tcp_lims[:3, 0], self.tcp_lims[:3, 1])
        rpy = np.clip(rpy, self.tcp_lims[3:, 0], self.tcp_lims[3:, 1])
        return pos, rpy

    def check_TCP_vel_lims(self, vels):
        """
        check whether action will take TCP outside of limits,
        zero any velocities that will.
        """
        cur_tcp_pos, cur_tcp_rpy, _, _, _ = self.get_current_TCP_pos_vel_workframe()

        # get bool arrays for if limits are exceeded and if velocity is in
        # the direction that's exceeded
        exceed_pos_llims = np.logical_and(cur_tcp_pos < self.tcp_lims[:3, 0], vels[:3] < 0)
        exceed_pos_ulims = np.logical_and(cur_tcp_pos > self.tcp_lims[:3, 1], vels[:3] > 0)
        exceed_rpy_llims = np.logical_and(cur_tcp_rpy < self.tcp_lims[3:, 0], vels[3:] < 0)
        exceed_rpy_ulims = np.logical_and(cur_tcp_rpy > self.tcp_lims[3:, 1], vels[3:] > 0)

        # combine all bool arrays into one
        exceeded_pos = np.logical_or(exceed_pos_llims, exceed_pos_ulims)
        exceeded_rpy = np.logical_or(exceed_rpy_llims, exceed_rpy_ulims)
        exceeded = np.concatenate([exceeded_pos, exceeded_rpy])

        # cap the velocities at 0 if limits are exceeded
        capped_vels = np.array(vels)
        capped_vels[np.array(exceeded)] = 0

        return capped_vels

    """
    ==================== Debug Tools ====================
    """

    def print_joint_pos_vel(self):
        joint_pos, joint_vel = self.get_current_joint_pos_vel()

        print("")
        print("joint pos: ", joint_pos)
        print("joint vel: ", joint_vel)

    def print_TCP_pos_vel(self):
        (
            tcp_pos,
            tcpy_rpy,
            tcp_orn,
            tcp_lin_vel,
            tcp_ang_vel,
        ) = self.get_current_TCP_pos_vel_worldframe()
        print("")
        print("tcp pos:     ", tcp_pos)
        print("tcp orn:     ", tcp_orn)
        print("tcp lin vel: ", tcp_lin_vel)
        print("tcp ang vel: ", tcp_ang_vel)

    def draw_workframe(self, lifetime=0.1):
        rpy = [0, 0, 0]
        self._pb.addUserDebugLine(
            self.workframe_pos,
            self.workframe_to_worldframe([0.1, 0, 0], rpy)[0],
            [1, 0, 0],
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            self.workframe_pos,
            self.workframe_to_worldframe([0, 0.1, 0], rpy)[0],
            [0, 1, 0],
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            self.workframe_pos,
            self.workframe_to_worldframe([0, 0, 0.1], rpy)[0],
            [0, 0, 1],
            lifeTime=lifetime,
        )

    def draw_tcp_box(self):
        self.tcp_lims[0, 0], self.tcp_lims[0, 1] = -0.1, +0.1  # x lims
        self.tcp_lims[1, 0], self.tcp_lims[1, 1] = -0.1, +0.1  # y lims
        self.tcp_lims[2, 0], self.tcp_lims[2, 1] = -0.1, +0.1  # z lims

        p1 = [
            self.workframe_pos[0] + self.tcp_lims[0, 0],
            self.workframe_pos[1] + self.tcp_lims[1, 1],
            self.workframe_pos[2],
        ]
        p2 = [
            self.workframe_pos[0] + self.tcp_lims[0, 0],
            self.workframe_pos[1] + self.tcp_lims[1, 0],
            self.workframe_pos[2],
        ]
        p3 = [
            self.workframe_pos[0] + self.tcp_lims[0, 1],
            self.workframe_pos[1] + self.tcp_lims[1, 0],
            self.workframe_pos[2],
        ]
        p4 = [
            self.workframe_pos[0] + self.tcp_lims[0, 1],
            self.workframe_pos[1] + self.tcp_lims[1, 1],
            self.workframe_pos[2],
        ]

        self._pb.addUserDebugLine(p1, p2, [1, 0, 0])
        self._pb.addUserDebugLine(p2, p3, [1, 0, 0])
        self._pb.addUserDebugLine(p3, p4, [1, 0, 0])
        self._pb.addUserDebugLine(p4, p1, [1, 0, 0])

    def test_workframe_transforms(self):
        init_pos = np.array([0, 0, 0])
        init_rpy = np.array([0, 0, 0])
        init_orn = np.array(self._pb.getQuaternionFromEuler(init_rpy))

        workframe_pos, workframe_orn = self.worldframe_to_workframe(init_pos, init_rpy)
        workframe_rpy = np.array(self._pb.getEulerFromQuaternion(workframe_orn))

        worldframe_pos, worldframe_rpy = self.workframe_to_worldframe(workframe_pos, workframe_rpy)
        worldframe_orn = np.array(self._pb.getQuaternionFromEuler(worldframe_rpy))

        float_formatter = "{:.4f}".format
        np.set_printoptions(formatter={"float_kind": float_formatter})
        print("")
        print("Init Position:       {}, Init RPY:       {}".format(init_pos, init_rpy))
        print("Workframe Position:  {}, Workframe RPY:  {}".format(workframe_pos, workframe_rpy))
        print("Worldframe Position: {}, Worldframe RPY: {}".format(worldframe_pos, worldframe_rpy))
        print(
            "Equal Pos: {}, Equal RPY: {}".format(
                np.isclose(init_pos, worldframe_pos).all(),
                np.isclose(init_rpy, worldframe_rpy).all(),
            )
        )

    def test_workvec_transforms(self):
        init_vec = np.random.uniform([0, 0, 1])
        work_vec = self.worldvec_to_workvec(init_vec)
        world_vec = self.workvec_to_worldvec(work_vec)

        float_formatter = "{:.4f}".format
        np.set_printoptions(formatter={"float_kind": float_formatter})
        print("")
        print("Init Vec:  {}".format(init_vec))
        print("Work Vec:  {}".format(work_vec))
        print("World Vec: {}".format(world_vec))
        print("Equal Vec: {}".format(np.isclose(init_vec, world_vec).all()))

    def test_workvel_transforms(self):
        init_lin_vel = np.random.uniform([0, 0, 1])
        init_ang_vel = np.random.uniform([0, 0, 1])
        work_lin_vel, work_ang_vel = self.worldvel_to_workvel(init_lin_vel, init_ang_vel)
        world_lin_vel, world_ang_vel = self.workvel_to_worldvel(work_lin_vel, work_ang_vel)

        float_formatter = "{:.4f}".format
        np.set_printoptions(formatter={"float_kind": float_formatter})
        print("")
        print("Init Lin Vel:  {}, Init Ang Vel:  {}".format(init_lin_vel, init_ang_vel))
        print("Work Lin Vel:  {},  Work Ang Vel: {}".format(work_lin_vel, work_ang_vel))
        print("World Lin Vel: {}, World Ang Vel: {}".format(world_lin_vel, world_ang_vel))
        print(
            "Equal Lin Vel: {}, Equal Ang Vel: {}".format(
                np.isclose(init_lin_vel, world_lin_vel).all(),
                np.isclose(init_ang_vel, world_ang_vel).all(),
            )
        )
