import time
import numpy as np

from tactile_sim.utils.setup_pb_utils import standard_argparse
from tactile_sim.utils.setup_pb_utils import connect_pybullet
from tactile_sim.utils.setup_pb_utils import load_standard_environment
from tactile_sim.utils.setup_pb_utils import set_debug_camera
from tactile_sim.utils.setup_pb_utils import add_tcp_user_control
from tactile_sim.utils.setup_pb_utils import add_joint_user_control
from tactile_sim.embodiments import create_embodiment
from tactile_sim.assets.default_rest_poses import rest_poses_dict


def demo_robot_control():

    timestep = 1/240.0
    show_gui = True

    parser = standard_argparse()
    parser.add_argument(
        '-c', '--control_mode',
        type=str,
        help="""Choose task from
                ['tcp_position_control', 'tcp_velocity_control', 'joint_position_control', 'joint_velocity_control'].""",
        default='tcp_velocity_control'
    )
    args = parser.parse_args()
    embodiment_type = args.embodiment_type
    arm_type = args.arm_type
    sensor_type = args.sensor_type
    control_mode = args.control_mode

    if control_mode not in ['tcp_position_control', 'tcp_velocity_control',
                            'joint_position_control', 'joint_velocity_control']:
        raise ValueError(f'Incorrect control_mode specified: {control_mode}')

    # define sensor parameters
    robot_arm_params = {
        "type": arm_type,
        "rest_poses": rest_poses_dict[arm_type],
    }

    tactile_sensor_params = {
        "type": sensor_type,
        "core": "fixed",
        "dynamics": {},  # {'stiffness': 50, 'damping': 100, 'friction': 10.0},
        "image_size": [128, 128],
        "turn_off_border": False,
        "show_tactile": False,
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
        'show_vision': False
    }

    pb = connect_pybullet(timestep, show_gui)
    load_standard_environment(pb)
    embodiment = create_embodiment(
        pb,
        embodiment_type,
        robot_arm_params,
        tactile_sensor_params,
        visual_sensor_params
    )
    set_debug_camera(pb, visual_sensor_params)

    if show_gui:
        if control_mode in ['tcp_position_control', 'tcp_velocity_control']:
            action_ids = add_tcp_user_control(pb)
        elif control_mode in ['joint_position_control', 'joint_velocity_control']:
            action_ids = add_joint_user_control(pb, embodiment.arm.control_joint_names)

    # set initial values
    targ_tcp_pose = np.array([0.35, 0.0, 0.2, -np.pi, 0.0, 0.0])
    targ_tcp_vels = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    targ_joint_positions = np.array(embodiment.arm.rest_poses[embodiment.arm.control_joint_ids])
    targ_joint_vels = np.array([0.0]*embodiment.arm.num_control_dofs)

    # move to initial pose
    if control_mode in ['tcp_position_control', 'tcp_velocity_control']:
        embodiment.arm.move_linear(targ_tcp_pose)
        action = np.zeros_like(targ_tcp_pose)
    elif control_mode in ['joint_position_control', 'joint_velocity_control']:
        embodiment.arm.move_joints(targ_joint_positions)
        action = np.zeros_like(targ_joint_positions)

    while pb.isConnected():

        # get an action from gui interface
        if show_gui:
            action = []
            for action_id in action_ids:
                action.append(pb.readUserDebugParameter(action_id))

        if control_mode == 'tcp_position_control':
            targ_tcp_pose += action
            embodiment.arm.move_linear(targ_tcp_pose)

        elif control_mode == 'tcp_velocity_control':
            targ_tcp_vels = action
            embodiment.arm.move_linear_vel(targ_tcp_vels)

        elif control_mode == 'joint_position_control':
            targ_joint_positions += action
            embodiment.arm.move_joints(targ_joint_positions)

        elif control_mode == 'joint_velocity_control':
            targ_joint_vels = action
            embodiment.arm.move_joints_vel(targ_joint_vels)

        # embodiment.arm.draw_ee()
        # embodiment.arm.draw_tcp()
        # embodiment.tactile_sensor.draw_camera_frame()
        # print(embodiment.tactile_sensor.get_contact_features())
        # print(embodiment.arm.get_joint_angles())

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
