from tactile_sim.embodiments.embodiments import ArmEmbodiment
from tactile_sim.embodiments.embodiments import TactileArmEmbodiment
from tactile_sim.embodiments.embodiments import VisualArmEmbodiment
from tactile_sim.embodiments.embodiments import VisuoTactileArmEmbodiment


def create_embodiment(pb, embodiment_type, robot_arm_params, tactile_sensor_params, visual_sensor_params):
    """
    Create a robot arm with attached sensors as our agent embodiment.
    """

    if embodiment_type == 'arm':
        # load a robot arm
        embodiment = ArmEmbodiment(
            pb,
            robot_arm_params=robot_arm_params
        )

    elif embodiment_type == 'tactile_arm':
        # load a robot arm with a tactile sensor attached
        embodiment = TactileArmEmbodiment(
            pb,
            robot_arm_params=robot_arm_params,
            tactile_sensor_params=tactile_sensor_params
        )

    elif embodiment_type == 'visual_arm':
        # load a robot arm with a static visual sensor
        embodiment = VisualArmEmbodiment(
            pb,
            robot_arm_params=robot_arm_params,
            visual_sensor_params=visual_sensor_params
        )

    elif embodiment_type == 'visuotactile_arm':
        # load a robot arm with a tactile sensor attached and a static visual sensor
        embodiment = VisuoTactileArmEmbodiment(
            pb,
            robot_arm_params=robot_arm_params,
            tactile_sensor_params=tactile_sensor_params,
            visual_sensor_params=visual_sensor_params
        )
    else:
        raise ValueError(f'Incorrect embodiment_type specified: {embodiment_type}')

    return embodiment
