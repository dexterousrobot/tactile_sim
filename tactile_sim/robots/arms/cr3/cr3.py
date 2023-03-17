from tactile_sim.robots.arms.base_robot_arm import BaseRobotArm


class CR3(BaseRobotArm):
    def __init__(
        self,
        pb,
        embodiment_id,
        tcp_link_id,
        link_name_to_index,
        joint_name_to_index,
        rest_poses
    ):
        super(CR3, self).__init__(
            pb, embodiment_id, tcp_link_id, link_name_to_index, joint_name_to_index, rest_poses
        )

        # set info specific to arm
        self.setup_ur5_info()

        # reset the arm to rest poses
        self.reset()

    def setup_ur5_info(self):
        """
        Set some of the parameters used when controlling the UR5
        """
        self.name = 'sim_cr3'
        self.max_force = 1000.0
        self.pos_gain = 1.0
        self.vel_gain = 1.0

        # joints which can be controlled (not fixed)
        self.control_joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]

        # get the control and calculate joint ids in list form, useful for pb array methods
        self.control_joint_ids = [self.joint_name_to_index[name] for name in self.control_joint_names]
        self.num_control_dofs = len(self.control_joint_ids)
