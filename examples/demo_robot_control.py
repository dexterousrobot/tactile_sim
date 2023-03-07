import time
import numpy as np
import argparse


from tactile_sim.utils.pybullet_utils import connect_pybullet
from tactile_sim.utils.pybullet_utils import load_standard_environment
from tactile_sim.utils.pybullet_utils import set_debug_camera
from tactile_sim.utils.pybullet_utils import add_user_control
from tactile_sim.embodiments.embodiments import ArmEmbodiment
from tactile_sim.embodiments.embodiments import TactileArmEmbodiment
from tactile_sim.embodiments.embodiments import VisualArmEmbodiment
from tactile_sim.embodiments.embodiments import VisuoTactileArmEmbodiment
from tactile_sim.assets.default_rest_poses import rest_poses_dict


def create_embodiment(pb, embodiment_type):
    """
    Create a robot arm with attached tactile sensor as our agent embodiment.
    """

    # define sensor parameters
    robot_arm_params = {
        "type": "ur5",
        # "type": "franka_panda",
        # "type": "kuka_iiwa",
        # "type": "cr3",
        # "type": "mg400",
    }

    tactile_sensor_params = {
        "type": "standard_tactip",
        # "type": "standard_digit",
        # "type": "standard_digitac",
        # "type": "mini_tactip",
        # "type": "flat_tactip",
        # "type": "right_angle_tactip",
        # "type": "right_angle_digit",
        # "type": "right_angle_digitac",

        "core": "no_core",
        "dynamics": {},  # {'stiffness': 50, 'damping': 100, 'friction': 10.0},
        "image_size": [128, 128],
        "turn_off_border": False,
        "show_tactile": True,
    }

    # on reset, joints are automatically set to the values defined here
    rest_poses = rest_poses_dict[robot_arm_params["type"]]
    robot_arm_params["rest_poses"] = rest_poses

    # define limits of the tool center point
    tcp_lims = np.ones(shape=(6, 2))
    tcp_lims[:, 0] *= -np.inf
    tcp_lims[:, 1] *= np.inf
    robot_arm_params["tcp_lims"] = tcp_lims

    # set debug camera position
    visual_sensor_params = {
        'image_size': [128, 128],
        'dist': 1.0,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.6, 0.0, 0.0525],
        'fov': 75.0,
        'near_val': 0.1,
        'far_val': 100.0
    }
    set_debug_camera(pb, visual_sensor_params)

    workframe = [0.35, 0.0, 0.1, -np.pi, 0.0, 0.0]
    show_gui = True

    if embodiment_type == 'arm':
        # load a robot arm
        embodiment = ArmEmbodiment(
            pb,
            workframe=workframe,
            robot_arm_params=robot_arm_params,
            show_gui=show_gui,
        )

    elif embodiment_type == 'tactile_arm':
        # load a robot arm with a tactile sensor attached
        embodiment = TactileArmEmbodiment(
            pb,
            workframe=workframe,
            robot_arm_params=robot_arm_params,
            tactile_sensor_params=tactile_sensor_params,
            show_gui=show_gui,
        )

    elif embodiment_type == 'visual_arm':
        # load a robot arm with a static visual sensor
        embodiment = VisualArmEmbodiment(
            pb,
            workframe=workframe,
            robot_arm_params=robot_arm_params,
            visual_sensor_params=visual_sensor_params,
            show_gui=show_gui,
        )

    elif embodiment_type == 'visuotactile_arm':
        # load a robot arm with a tactile sensor attached and a static visual sensor
        embodiment = VisuoTactileArmEmbodiment(
            pb,
            workframe=workframe,
            robot_arm_params=robot_arm_params,
            tactile_sensor_params=tactile_sensor_params,
            visual_sensor_params=visual_sensor_params,
            show_gui=show_gui,
        )

    return embodiment


def demo_robot_control():

    timestep = 1/240.0

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-e', '--embodiment_type',
        type=str,
        help="Choose task from ['arm', 'tactile_arm', 'visual_arm', 'visuotactile_arm'].",
        default='visuotactile_arm'
    )
    args = parser.parse_args()
    embodiment_type = args.embodiment_type

    pb = connect_pybullet(timestep)
    load_standard_environment(pb)
    embodiment = create_embodiment(pb, embodiment_type)
    action_ids = add_user_control(pb)

    # move to workframe
    embodiment.arm.tcp_direct_workframe_move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    embodiment.arm.blocking_move(max_steps=10000, constant_vel=0.001)

    while pb.isConnected():

        # get an action from gui interface
        a = []
        for action_id in action_ids:
            a.append(pb.readUserDebugParameter(action_id))

        # apply the actions
        embodiment.arm.apply_action(a)

        # embodiment.draw_ee()
        # embodiment.draw_tcp()
        # embodiment.arm.draw_workframe()
        # embodiment.sensor.draw_sensor_frame()
        # embodiment.sensor.draw_camera_frame()

        if embodiment_type in ['tactile_arm', 'visuotactile_arm']:
            embodiment.get_tactile_observation()

        if embodiment_type in ['visual_arm', 'visuotactile_arm']:
            embodiment.get_visual_observation()

        embodiment.arm.step_sim()
        time.sleep(timestep)

        q_key = ord("q")
        keys = pb.getKeyboardEvents()
        if q_key in keys and keys[q_key] & pb.KEY_WAS_TRIGGERED:
            exit()


if __name__ == "__main__":
    demo_robot_control()
