import numpy as np
from tactile_sim.utils.pybullet_draw_utils import draw_link_frame


class BaseRobotArm:
    def __init__(
        self,
        pb,
        embodiment_id,
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

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()

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

    def get_current_TCP_pos_vel(self):
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

    def get_tcp_pose(self):
        """
        Returns pose of the Tool Center Point in world frame
        """
        (
                cur_TCP_pos,
                cur_TCP_rpy,
                _,
                _,
                _,
        ) = self.get_current_TCP_pos_vel()
        return np.array([*cur_TCP_pos, *cur_TCP_rpy])

    def get_joint_angles(self):
        """
        Returns pose of the Tool Center Point in world frame
        """
        joint_pos, _ = self.get_current_joint_pos_vel()
        return np.array(joint_pos)

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

    def set_target_tcp_pose(self, target_pos, target_rpy):
        """
        Go directly to a position specified relative to the workframe
        """

        # transform from work_frame to world_frame
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
        self.target_joints = np.array(joint_poses)

    def set_target_joints(self, joint_poses):
        """
        Go directly to a specified joint configuration
        """

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
        self.target_joints = np.array(joint_poses)

    def step_sim(self):
        """
        Take a step of the simulation whilst applying neccessary forces
        """

        # compensate for the effect of gravity
        self.apply_gravity_compensation()

        # step the simulation
        self._pb.stepSimulation()

    def blocking_move(
        self,
        max_steps=1000,
        constant_vel=None,
        j_pos_tol=0.1,
        j_vel_tol=0.1,
    ):
        """
        step the simulation until a target position has been reached or the max
        number of steps has been reached
        """
        # get target position
        targ_j_pos = self.target_joints

        for i in range(max_steps):

            # get the current position and veloicities (worldframe)
            (
                cur_TCP_pos,
                cur_TCP_rpy,
                cur_TCP_orn,
                _,
                _,
            ) = self.get_current_TCP_pos_vel()

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
            j_vel_err = np.sum(np.abs(cur_j_vel))
            j_pos_err = np.sum(np.abs(targ_j_pos - cur_j_pos))

            # break if the pose error is small enough
            # and the velocity is low enough
            if (j_pos_err < j_pos_tol) and (j_vel_err < j_vel_tol):
                break

    def apply_blocking_move(self, quick_mode=False):
        if not quick_mode:
            # slow but more realistic moves
            self.blocking_move(
                max_steps=10000,
                constant_vel=0.00025,
                j_pos_tol=1e-6,
                j_vel_tol=1e-3,
            )

        else:
            # fast but unrealistic moves (bigger_moves = worse performance)
            self.blocking_move(
                max_steps=1000,
                constant_vel=None,
                j_pos_tol=1e-6,
                j_vel_tol=1e-3,
            )

    def move_linear(self, targ_pose, quick_mode=False):
        targ_pos, targ_rpy = targ_pose[:3], targ_pose[3:]
        self.set_target_tcp_pose(targ_pos, targ_rpy)
        self.apply_blocking_move(quick_mode=quick_mode)

    def move_joints(self, targ_joint_angles, quick_mode=False):
        self.set_target_joints(targ_joint_angles)
        self.apply_blocking_move(quick_mode=quick_mode)

    def check_TCP_pos_lims(self, pos, rpy):
        """
        cap the pos at the TCP limits specified
        """
        pos = np.clip(pos, self.tcp_lims[:3, 0], self.tcp_lims[:3, 1])
        rpy = np.clip(rpy, self.tcp_lims[3:, 0], self.tcp_lims[3:, 1])
        return pos, rpy

    """
    ==================== Debug Tools ====================
    """

    def draw_ee(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.link_name_to_index["ee_link"], lifetime=lifetime)

    def draw_tcp(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.tcp_link_id, lifetime=lifetime)
