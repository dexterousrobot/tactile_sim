import time
import numpy as np
import argparse


from tactile_sim.utils.pybullet_utils import connect_pybullet
from tactile_sim.utils.pybullet_utils import load_standard_environment
from tactile_sim.utils.pybullet_utils import set_debug_camera
from tactile_sim.utils.pybullet_utils import add_user_control
from tactile_sim.embodiments import create_embodiment
from tactile_sim.assets.default_rest_poses import rest_poses_dict


def demo_robot_control():

    timestep = 1/240.0
    show_gui = True

    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-e', '--embodiment_type',
        type=str,
        help="Choose task from ['arm', 'tactile_arm', 'visual_arm', 'visuotactile_arm'].",
        default='visuotactile_arm'
    )
    parser.add_argument(
        '-a', '--arm_type',
        type=str,
        help="Choose task from ['ur5', 'franka_panda', 'kuka_iiwa', 'cr3', 'mg400'].",
        default='ur5'
    )
    parser.add_argument(
        '-s', '--sensor_type',
        type=str,
        help="""Choose task from
                ['standard_tactip', 'standard_digit', 'standard_digitac', 'mini_tactip', 'flat_tactip',
                'right_angle_tactip', 'right_angle_digit', 'right_angle_digitac'].""",
        default='right_angle_tactip'
    )
    args = parser.parse_args()
    embodiment_type = args.embodiment_type
    arm_type = args.arm_type
    sensor_type = args.sensor_type

    # define sensor parameters
    robot_arm_params = {
        "type": arm_type,
        "rest_poses": rest_poses_dict[arm_type],
        "tcp_lims": np.column_stack([-np.inf * np.ones(6), np.inf * np.ones(6)]),
    }

    tactile_sensor_params = {
        "type": sensor_type,
        "core": "no_core",
        "dynamics": {},  # {'stiffness': 50, 'damping': 100, 'friction': 10.0},
        "image_size": [128, 128],
        "turn_off_border": False,
        "show_tactile": True,
    }

    # set debug camera position
    visual_sensor_params = {
        'image_size': [128, 128],
        'dist': 1.0,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.6, 0.0, 0.0525],
        'fov': 75.0,
        'near_val': 0.1,
        'far_val': 100.0,
        'show_vision': True
    }

    workframe = [0.35, 0.0, 0.1, -np.pi, 0.0, 0.0]

    pb = connect_pybullet(timestep, show_gui)
    load_standard_environment(pb)
    embodiment = create_embodiment(
        pb,
        workframe,
        embodiment_type,
        robot_arm_params,
        tactile_sensor_params,
        visual_sensor_params
    )
    set_debug_camera(pb, visual_sensor_params)

    if show_gui:
        action_ids = add_user_control(pb)

    # move to workframe
    embodiment.arm.tcp_direct_workframe_move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    embodiment.arm.blocking_move(max_steps=1000, constant_vel=0.005)

    while pb.isConnected():

        # get an action from gui interface
        if show_gui:
            a = []
            for action_id in action_ids:
                a.append(pb.readUserDebugParameter(action_id))
        else:
            a = np.zeros(6)

        # apply the actions
        embodiment.arm.apply_action(a)

        # embodiment.arm.draw_ee()
        # embodiment.arm.draw_tcp()
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
