import pybullet as p
import pybullet_utils.bullet_client as bc
from tactile_sim.assets import add_assets_path


def connect_pybullet(timestep):
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


def load_standard_environment(pb):
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


def set_debug_camera(pb, debug_camera_params):
    pb.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    pb.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    pb.resetDebugVisualizerCamera(
        debug_camera_params['dist'],
        debug_camera_params['yaw'],
        debug_camera_params['pitch'],
        debug_camera_params['pos']
    )


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
