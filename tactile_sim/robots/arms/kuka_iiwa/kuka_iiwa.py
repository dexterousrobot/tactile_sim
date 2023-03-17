from tactile_sim.robots.arms.base_robot_arm import BaseRobotArm


class KukaIiwa(BaseRobotArm):
    def __init__(
        self,
        pb,
        embodiment_id,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses
    ):
        super(KukaIiwa, self).__init__(
            pb, embodiment_id, tcp_link_id, link_name_to_index, joint_name_to_index, rest_poses
        )

        # set info specific to arm
        self.setup_kuka_info()

        # reset the arm to rest poses
        self.reset()

    def setup_kuka_info(self):
        """
        Set some of the parameters used when controlling the Franka Panda
        """
        self.name = 'sim_kuka_iiwa'
        self.max_force = 1000.0
        self.pos_gain = 1.0
        self.vel_gain = 1.0

        self.num_joints = self._pb.getNumJoints(self.embodiment_id)

        # joints which can be controlled (not fixed)
        self.control_joint_names = [
            "lbr_iiwa_joint_1",
            "lbr_iiwa_joint_2",
            "lbr_iiwa_joint_3",
            "lbr_iiwa_joint_4",
            "lbr_iiwa_joint_5",
            "lbr_iiwa_joint_6",
            "lbr_iiwa_joint_7",
        ]

        # get the control and calculate joint ids in list form, useful for pb array methods
        self.control_joint_ids = [self.joint_name_to_index[name] for name in self.control_joint_names]
        self.num_control_dofs = len(self.control_joint_ids)
