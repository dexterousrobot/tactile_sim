from tactile_sim.robots.arms.base_robot_arm import BaseRobotArm


class FrankaPanda(BaseRobotArm):
    def __init__(
        self,
        pb,
        embodiment_id,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses
    ):
        super(FrankaPanda, self).__init__(
            pb, embodiment_id, tcp_link_id, link_name_to_index, joint_name_to_index, rest_poses
        )

        # set info specific to arm
        self.setup_panda_info()

        # reset the arm to rest poses
        self.reset()

    def setup_panda_info(self):
        """
        Set some of the parameters used when controlling the Franka Panda
        """
        self.name = 'sim_franka_panda'
        self.max_force = 1000.0
        self.pos_gain = 1.0
        self.vel_gain = 1.0

        self.num_joints = self._pb.getNumJoints(self.embodiment_id)

        # joints which can be controlled (not fixed)
        self.control_joint_names = [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ]

        # get the control and calculate joint ids in list form, useful for pb array methods
        self.control_joint_ids = [self.joint_name_to_index[name] for name in self.control_joint_names]
        self.num_control_dofs = len(self.control_joint_ids)
