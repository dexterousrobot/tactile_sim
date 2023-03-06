import time
import numpy as np
import math
import pybullet as p
import pybullet_utils.bullet_client as bc

from tactile_sim.assets import add_assets_path
from tactile_sim.embodiments.arm_sensor_embodiment import ArmSensorEmbodiment
from tactile_sim.assets.default_rest_poses import rest_poses_dict


def setup_pybullet(timestep):
    """
    Create a pyullet instance with set physics params.
    """
    pb = bc.BulletClient(connection_mode=p.GUI)
    pb.setGravity(0, 0, -10)
    pb.setPhysicsEngineParameter(
        fixedTimeStep=timestep,
        numSolverIterations=300,
        numSubSteps=1,
        contactBreakingThreshold=0.0005,
        erp=0.05,
        contactERP=0.05,
        frictionERP=0.2,
        solverResidualThreshold=1e-7,
        contactSlop=0.001,
        globalCFM=0.0001,
    )
    return pb


def load_environment(pb):
    """
    Load a standard environment with a plane and a table.
    """
    pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/plane/plane.urdf"),
        [0, 0, -0.625],
    )
    pb.loadURDF(
        add_assets_path("shared_assets/environment_objects/table/table.urdf"),
        [0.50, 0.00, -0.625],
        [0.0, 0.0, 0.0, 1.0],
    )
    # set debug camera position
    cam_params = {
        'image_size': [512, 512],
        'dist': 1.0,
        'yaw': 90.0,
        'pitch': -25.0,
        'pos': [0.6, 0.0, 0.0525],
        'fov': 75.0,
        'near_val': 0.1,
        'far_val': 100.0
    }

    pb.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    pb.resetDebugVisualizerCamera(
        cam_params['dist'],
        cam_params['yaw'],
        cam_params['pitch'],
        cam_params['pos']
    )


def load_embodiment(pb):
    """
    Create a robot arm with attached tactile sensor as our agent embodiment.
    """

    # robot_arm_type = "ur5"
    # robot_arm_type = "franka_panda"
    # robot_arm_type = "kuka_iiwa"
    robot_arm_type = "cr3"
    # robot_arm_type = "mg400"

    # define sensor parameters
    sensor_type = "standard_tactip"
    # sensor_type = "standard_digit"
    # sensor_type = "standard_digitac"
    # sensor_type = "mini_tactip"
    # sensor_type = "flat_tactip"
    # sensor_type = "right_angle_tactip"
    # sensor_type = "right_angle_digit"
    # sensor_type = "right_angle_digitac"

    sensor_params = {
        "type": sensor_type,
        "core": "no_core",
        "dynamics": {},  # {'stiffness': 50, 'damping': 100, 'friction': 10.0},
        "image_size": [128, 128],
        "turn_off_border": False,
    }

    # on reset, joints are automatically set to the values defined here
    rest_poses = rest_poses_dict[robot_arm_type]

    # define limits of the tool center point
    tcp_lims = np.zeros(shape=(6, 2))
    tcp_lims[0, 0], tcp_lims[0, 1] = -np.inf, +np.inf  # x lims
    tcp_lims[1, 0], tcp_lims[1, 1] = -np.inf, +np.inf  # y lims
    tcp_lims[2, 0], tcp_lims[2, 1] = -np.inf, +np.inf  # z lims
    tcp_lims[3, 0], tcp_lims[3, 1] = -np.inf, +np.inf  # roll lims
    tcp_lims[4, 0], tcp_lims[4, 1] = -np.inf, +np.inf  # pitch lims
    tcp_lims[5, 0], tcp_lims[5, 1] = -np.inf, +np.inf  # yaw lims

    # define sensor parameters
    robot_arm_params = {
        "type": robot_arm_type,
        "rest_poses": rest_poses,
        "tcp_lims": tcp_lims,
    }

    workframe = [0.35, 0.0, 0.1, -np.pi, 0.0, 0.0]
    show_gui = True
    show_tactile = True

    # load the ur5 with a tactip attached
    embodiment = ArmSensorEmbodiment(
        pb,
        workframe=workframe,
        robot_arm_params=robot_arm_params,
        sensor_params=sensor_params,
        show_gui=show_gui,
        show_tactile=show_tactile,
    )

    return embodiment


def add_user_control(pb):
    # create controllable parameters on GUI
    action_ids = []
    min_action, max_action = -0.25, 0.25

    action_ids.append(
        pb.addUserDebugParameter("dx", min_action, max_action, 0)
    )
    action_ids.append(
        pb.addUserDebugParameter("dy", min_action, max_action, 0)
    )
    action_ids.append(
        pb.addUserDebugParameter("dz", min_action, max_action, 0)
    )
    action_ids.append(
        pb.addUserDebugParameter("dRX", min_action, max_action, 0)
    )
    action_ids.append(
        pb.addUserDebugParameter("dRY", min_action, max_action, 0)
    )
    action_ids.append(
        pb.addUserDebugParameter("dRZ", min_action, max_action, 0)
    )
    return action_ids


def demo_robot_control():
    timestep = 1/240.0
    pb = setup_pybullet(timestep)
    load_environment(pb)
    embodiment = load_embodiment(pb)

    action_ids = add_user_control(pb)

    # move to workframe
    embodiment.arm.tcp_direct_workframe_move([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    embodiment.blocking_move(max_steps=10000, constant_vel=0.001)

    while pb.isConnected():

        # get an action from gui interface
        a = []
        for action_id in action_ids:
            a.append(pb.readUserDebugParameter(action_id))

        # apply the actions
        embodiment.apply_action(a)

        # embodiment.draw_ee()
        # embodiment.draw_tcp()
        # embodiment.arm.draw_workframe()
        # embodiment.arm.print_joint_pos_vel()
        # embodiment.sensor.draw_sensor_frame()
        # embodiment.sensor.draw_camera_frame()

        embodiment.get_tactile_observation()

        #  print joint states
        # joint_ids = list(range(embodiment.num_joints))
        # joint_states = pb.getJointStates(embodiment.embodiment_id, joint_ids)
        # cur_joint_pos = [joint_states[i][0] for i in joint_ids]
        # print(cur_joint_pos)

        embodiment.step_sim()
        time.sleep(timestep)

        q_key = ord("q")
        keys = pb.getKeyboardEvents()
        if q_key in keys and keys[q_key] & pb.KEY_WAS_TRIGGERED:
            exit()


if __name__ == "__main__":
    demo_robot_control()
