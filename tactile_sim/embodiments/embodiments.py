import os
import numpy as np

from tactile_sim.assets import add_assets_path
from tactile_sim.robots.arms import arm_mapping
from tactile_sim.sensors.tactile_sensor import TactileSensor
from tactile_sim.sensors.vision_sensor import VisionSensor
from tactile_sim.utils.pybullet_draw_utils import draw_link_frame


class ArmEmbodiment:
    def __init__(
        self,
        pb,
        workframe=[0.65, 0.0, 0.0525, -np.pi, 0.0, 0.0],
        robot_arm_params={},
        show_gui=True,
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            workframe=workframe,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
            tcp_lims=robot_arm_params['tcp_lims']
        )

    def load_urdf(self):
        """
        Load the robot arm model into pybullet
        """
        self.base_pos = [0, 0, 0]
        self.base_rpy = [0, 0, 0]
        self.base_orn = self._pb.getQuaternionFromEuler(self.base_rpy)
        asset_name = os.path.join(
            "robot_arm_assets",
            self.arm_type,
            "urdfs",
            self.arm_type + ".urdf",
        )

        self.embodiment_id = self._pb.loadURDF(
            add_assets_path(asset_name), self.base_pos, self.base_orn, useFixedBase=True
        )

        # create dicts for mapping link/joint names to corresponding indices
        self.num_joints, self.link_name_to_index, self.joint_name_to_index = self.create_link_joint_mappings(
            self.embodiment_id)

        # get the link and tcp IDs
        self.ee_link_id = self.link_name_to_index["ee_link"]
        self.tcp_link_id = self.link_name_to_index["ee_link"]

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
        self.blocking_move(max_steps=10000, constant_vel=0.001)

    def full_reset(self):
        self.load_urdf()
        self.sensor.turn_off_collisions()

    def draw_ee(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.ee_link_id, lifetime=lifetime)

    def draw_tcp(self, lifetime=0.1):
        draw_link_frame(self.embodiment_id, self.tcp_link_id, lifetime=lifetime)


class TactileArmEmbodiment(ArmEmbodiment):
    def __init__(
        self,
        pb,
        workframe=[0.65, 0.0, 0.0525, -np.pi, 0.0, 0.0],
        robot_arm_params={},
        tactile_sensor_params={},
        show_gui=True,
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]
        self.tactile_sensor_type = tactile_sensor_params["type"]

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            workframe=workframe,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
            tcp_lims=robot_arm_params['tcp_lims']
        )

        # connect a tactile sensor
        self.tactile_sensor = TactileSensor(
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            image_size=tactile_sensor_params["image_size"],
            turn_off_border=tactile_sensor_params["turn_off_border"],
            sensor_type=tactile_sensor_params["type"],
            sensor_core=tactile_sensor_params["core"],
            sensor_dynamics=tactile_sensor_params["dynamics"],
            show_tactile=tactile_sensor_params["show_tactile"],
            sensor_num=1,
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
            self.arm_type + "_" + self.tactile_sensor_type + ".urdf",
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

    def reset(self, reset_TCP_pos, reset_TCP_rpy):
        """
        Reset the pose of the arm and sensor
        """
        self.arm.reset()
        self.tactile_sensor.reset()

        # move to the initial position
        self.arm.tcp_direct_workframe_move(reset_TCP_pos, reset_TCP_rpy)
        self.blocking_move(max_steps=1000, constant_vel=0.001)

    def full_reset(self):
        self.load_urdf()
        self.tactile_sensor.turn_off_collisions()

    def get_tactile_observation(self):
        return self.tactile_sensor.get_observation()


class VisualArmEmbodiment(ArmEmbodiment):
    def __init__(
        self,
        pb,
        workframe=[0.65, 0.0, 0.0525, -np.pi, 0.0, 0.0],
        robot_arm_params={},
        visual_sensor_params={},
        show_gui=True,
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            workframe=workframe,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
            tcp_lims=robot_arm_params['tcp_lims']
        )

        # connect a static vision sensor
        self.vision_sensor = VisionSensor(
            pb,
            sensor_num=1,
            **visual_sensor_params
        )

    def full_reset(self):
        self.load_urdf()

    def get_visual_observation(self):
        return self.vision_sensor.get_observation()


class VisuoTactileArmEmbodiment(TactileArmEmbodiment):
    def __init__(
        self,
        pb,
        workframe=[0.65, 0.0, 0.0525, -np.pi, 0.0, 0.0],
        robot_arm_params={},
        tactile_sensor_params={},
        visual_sensor_params={},
        show_gui=True,
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]
        self.tactile_sensor_type = tactile_sensor_params["type"]

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            workframe=workframe,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
            tcp_lims=robot_arm_params['tcp_lims']
        )

        # connect a tactile sensor
        self.tactile_sensor = TactileSensor(
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            image_size=tactile_sensor_params["image_size"],
            turn_off_border=tactile_sensor_params["turn_off_border"],
            sensor_type=tactile_sensor_params["type"],
            sensor_core=tactile_sensor_params["core"],
            sensor_dynamics=tactile_sensor_params["dynamics"],
            show_tactile=tactile_sensor_params["show_tactile"],
            sensor_num=1,
        )

        # connect a static vision sensor
        self.vision_sensor = VisionSensor(
            pb,
            sensor_num=1,
            **visual_sensor_params
        )

    def get_visual_observation(self):
        return self.vision_sensor.get_observation()
