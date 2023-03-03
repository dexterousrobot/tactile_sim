import os
import sys
import numpy as np

from tactile_sim.assets import add_assets_path
from tactile_sim.robots.arms import arm_mapping
from tactile_sim.sensors.base_tactile_sensor import TactileSensor


class ArmSensorEmbodiment:
    def __init__(
        self,
        pb,
        workframe=[0.65, 0.0, 0.0525, -np.pi, 0.0, 0.0],
        robot_arm_params={},
        sensor_params={},
        show_gui=True,
        show_tactile=True,
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]
        self.sensor_type = sensor_params["type"]

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            workframe=workframe,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
            tcp_lims=robot_arm_params['tcp_lims']
        )

        # connect the sensor the tactip
        self.sensor = TactileSensor(
            pb,
            embodiment_id=self.embodiment_id,
            link_name_to_index=self.link_name_to_index,
            image_size=sensor_params["image_size"],
            turn_off_border=sensor_params["turn_off_border"],
            sensor_type=sensor_params["type"],
            sensor_core=sensor_params["core"],
            sensor_dynamics=sensor_params["dynamics"],
            show_tactile=show_tactile,
            sensor_num=1
        )

    def load_urdf(self):
        """
        Load the robot arm model into pybullet
        """
        self.base_pos = [0, 0, 0]
        self.base_rpy = [0, 0, 0]
        self.base_orn = self._pb.getQuaternionFromEuler(self.base_rpy)
        asset_name = os.path.join(
            "embodiment_assets",
            "combined_urdfs",
            self.arm_type + "_" + self.sensor_type + ".urdf",
        )

        self.embodiment_id = self._pb.loadURDF(
            add_assets_path(asset_name), self.base_pos, self.base_orn, useFixedBase=True
        )

        # create dicts for mapping link/joint names to corresponding indices
        self.num_joints, self.link_name_to_index, self.joint_name_to_index = self.create_link_joint_mappings(
            self.embodiment_id)

        # get the link and tcp IDs
        self.ee_link_id = self.link_name_to_index["ee_link"]
        self.tcp_link_id = self.link_name_to_index["tcp_link"]

    def create_link_joint_mappings(self, urdf_id):

        num_joints = self._pb.getNumJoints(urdf_id)

        # pull relevent info for controlling the robot
        joint_name_to_index = {}
        link_name_to_index = {}
        for i in range(num_joints):
            info = self._pb.getJointInfo(urdf_id, i)
            joint_name = info[1].decode("utf-8")
            link_name = info[12].decode("utf-8")
            joint_name_to_index[joint_name] = i
            link_name_to_index[link_name] = i

        return num_joints, link_name_to_index, joint_name_to_index

    def reset(self, reset_TCP_pos, reset_TCP_rpy):
        """
        Reset the pose of the UR5 and sensor
        """
        self.arm.reset()
        self.sensor.reset()

        # move to the initial position
        self.arm.tcp_direct_workframe_move(reset_TCP_pos, reset_TCP_rpy)
        self.blocking_move(max_steps=1000, constant_vel=0.001)

    def full_reset(self):
        self.load_urdf()
        self.sensor.turn_off_collisions()

    def step_sim(self):
        """
        Take a step of the simulation whilst applying neccessary forces
        """

        # compensate for the effect of gravity
        self.arm.apply_gravity_compensation()

        # step the simulation
        self._pb.stepSimulation()

        # debugging
        # self.arm.draw_EE()
        # self.arm.draw_TCP() # only works with visuals enabled in urdf file
        # self.arm.draw_workframe()
        # self.arm.draw_TCP_box()
        # self.arm.print_joint_pos_vel()
        # self.arm.print_TCP_pos_vel()
        # self.arm.test_workframe_transforms()
        # self.arm.test_workvec_transforms()
        # self.arm.test_workvel_transforms()
        # self.sensor.draw_camera_frame()
        # self.sensor.draw_sensor_frame()

    def apply_action(
        self,
        motor_commands,
        control_mode="TCP_velocity_control",
        velocity_action_repeat=1,
        max_steps=100,
    ):

        if control_mode == "TCP_position_control":
            self.arm.tcp_position_control(motor_commands)

        elif control_mode == "TCP_velocity_control":
            self.arm.tcp_velocity_control(motor_commands)

        elif control_mode == "joint_velocity_control":
            self.arm.joint_velocity_control(motor_commands)

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
        targ_pos = self.arm.target_pos_worldframe
        targ_orn = self.arm.target_orn_worldframe
        targ_j_pos = self.arm.target_joints

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
            ) = self.arm.get_current_TCP_pos_vel_worldframe()

            # get the current joint positions and velocities
            cur_j_pos, cur_j_vel = self.arm.get_current_joint_pos_vel()

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
                    self.arm.control_joint_ids,
                    self._pb.POSITION_CONTROL,
                    targetPositions=step_j,
                    targetVelocities=[0.0] * self.arm.num_control_dofs,
                    positionGains=[self.arm.pos_gain] * self.arm.num_control_dofs,
                    velocityGains=[self.arm.vel_gain] * self.arm.num_control_dofs
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

    def get_tactile_observation(self):
        return self.sensor.get_observation()

    def draw_ee(self, lifetime=0.1):
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0.1, 0, 0],
            [1, 0, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.ee_link_id,
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0.1, 0],
            [0, 1, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.ee_link_id,
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0, 0.1],
            [0, 0, 1],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.ee_link_id,
            lifeTime=lifetime,
        )

    def draw_tcp(self, lifetime=0.1):
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0.1, 0, 0],
            [1, 0, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tcp_link_id,
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0.1, 0],
            [0, 1, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tcp_link_id,
            lifeTime=lifetime,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0, 0.1],
            [0, 0, 1],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tcp_link_id,
            lifeTime=lifetime,
        )
