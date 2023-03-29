import os

from tactile_sim.assets import add_assets_path
from tactile_sim.robots.arms import arm_mapping
from tactile_sim.sensors.tactile_sensor import TactileSensor
from tactile_sim.sensors.vision_sensor import VisionSensor


class ArmEmbodiment:
    def __init__(
        self,
        pb,
        robot_arm_params={}
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]

        if "tcp_link_name" in robot_arm_params:
            self.tcp_link_name = robot_arm_params["tcp_link_name"]
        else:
            self.tcp_link_name = "ee_link"

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
        )

    def close(self):
        if self._pb.isConnected():
            self._pb.disconnect()

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
        self.tcp_link_id = self.link_name_to_index[self.tcp_link_name]

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

    def reset(self, reset_tcp_pose):
        """
        Reset the pose of the arm and sensor
        """
        self.arm.reset()

        # move to the initial position
        self.arm.move_linear(reset_tcp_pose, quick_mode=True)

    def full_reset(self):
        self.load_urdf()
        self.sensor.turn_off_collisions()


class TactileArmEmbodiment(ArmEmbodiment):
    def __init__(
        self,
        pb,
        robot_arm_params={},
        tactile_sensor_params={}
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]
        self.tactile_sensor_type = tactile_sensor_params["type"]

        if "tcp_link_name" in robot_arm_params:
            self.tcp_link_name = robot_arm_params["tcp_link_name"]
        else:
            self.tcp_link_name = "ee_link"

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
        )

        # connect a tactile sensor
        self.tactile_sensor = TactileSensor(
            pb,
            embodiment_id=self.embodiment_id,
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
        self.tcp_link_id = self.link_name_to_index[self.tcp_link_name]

    def reset(self, reset_tcp_pose):
        """
        Reset the pose of the arm and sensor
        """
        self.arm.reset()
        self.tactile_sensor.reset()

        # move to the initial position
        self.arm.move_linear(reset_tcp_pose, quick_mode=True)

    def full_reset(self):
        self.load_urdf()
        self.tactile_sensor.turn_off_collisions()

    def get_tactile_observation(self):
        return self.tactile_sensor.get_observation()


class VisualArmEmbodiment(ArmEmbodiment):
    def __init__(
        self,
        pb,
        robot_arm_params={},
        visual_sensor_params={}
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]

        if "tcp_link_name" in robot_arm_params:
            self.tcp_link_name = robot_arm_params["tcp_link_name"]
        else:
            self.tcp_link_name = "ee_link"

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
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
        robot_arm_params={},
        tactile_sensor_params={},
        visual_sensor_params={}
    ):

        self._pb = pb
        self.arm_type = robot_arm_params["type"]
        self.tactile_sensor_type = tactile_sensor_params["type"]

        if "tcp_link_name" in robot_arm_params:
            self.tcp_link_name = robot_arm_params["tcp_link_name"]
        else:
            self.tcp_link_name = "ee_link"

        # load the urdf file
        self.load_urdf()

        # instantiate a robot arm
        self.arm = arm_mapping[self.arm_type](
            pb,
            embodiment_id=self.embodiment_id,
            tcp_link_id=self.tcp_link_id,
            link_name_to_index=self.link_name_to_index,
            joint_name_to_index=self.joint_name_to_index,
            rest_poses=robot_arm_params['rest_poses'],
        )

        # connect a tactile sensor
        self.tactile_sensor = TactileSensor(
            pb,
            embodiment_id=self.embodiment_id,
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
