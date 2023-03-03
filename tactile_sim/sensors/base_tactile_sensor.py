import os
import numpy as np
import cv2

from tactile_sim.assets import add_assets_path


SENSOR_TYPES = [
    'standard_tactip',
    'standard_digit',
    'standard_digitac',
    'flat_tactip',
    'mini_tactip',
    'right_angle_tactip',
    'right_angle_digit',
    'right_angle_digitac',
    'right_angle_flat_tactip',
    'right_angle_mini_tactip',
]


class TactileSensor:
    def __init__(
        self,
        pb,
        embodiment_id,
        link_name_to_index,
        image_size=[128, 128],
        turn_off_border=False,
        sensor_type="standard",
        sensor_core="no_core",
        sensor_dynamics={},
        show_tactile=True,
        sensor_num=int(0),
    ):

        self._pb = pb
        self.embodiment_id = embodiment_id
        self.show_tactile = show_tactile
        self.sensor_type = sensor_type
        self.sensor_family = sensor_type.split('_')[-1]
        self.sensor_core = sensor_core
        self.sensor_dynamics = sensor_dynamics
        self.image_size = image_size
        self.turn_off_border = turn_off_border
        self.sensor_num = sensor_num

        # get relevent link ids for turning off collisions, connecting camera, etc
        self.tactile_link_ids = {}
        self.tactile_link_ids['body'] = link_name_to_index[self.sensor_family + "_body_link"]
        self.tactile_link_ids['tip'] = link_name_to_index[self.sensor_family + "_tip_link"]
        if self.sensor_family + "_adapter_link" in link_name_to_index.keys():
            self.tactile_link_ids['adapter'] = link_name_to_index["tactip_adapter_link"]

        self.load_reference_images()
        self.setup_camera_info()
        # self.save_reference_images()
        self.connect()
        self.turn_off_collisions()

    def turn_off_collisions(self):
        """
        Turn off collisions between sensor base and rest of the envs,
        useful for speed of training due to mininmising collisions
        """

        # turn off body collisions
        self._pb.setCollisionFilterGroupMask(self.embodiment_id, self.tactile_link_ids["body"], 0, 0)

        # turn off adapter collisions
        if "adapter" in self.tactile_link_ids.keys():
            self._pb.setCollisionFilterGroupMask(self.embodiment_id, self.tactile_link_ids["adapter"], 0, 0)

        # turn of "core" collisions
        if self.sensor_core == "no_core":
            self._pb.setCollisionFilterGroupMask(self.embodiment_id, self.tactile_link_ids["tip"], 0, 0)

    def load_reference_images(self):
        # get saved reference images
        border_images_path = add_assets_path("reference_images")

        saved_file_dir = os.path.join(
            border_images_path,
            self.sensor_type,
            str(self.image_size[0]) + "x" + str(self.image_size[1]),
        )

        nodef_gray_savefile = os.path.join(saved_file_dir, "nodef_gray.npy")
        nodef_dep_savefile = os.path.join(saved_file_dir, "nodef_dep.npy")
        border_mask_savefile = os.path.join(saved_file_dir, "border_mask.npy")

        # load border images from simulation
        self.no_deformation_gray = np.load(nodef_gray_savefile)
        self.no_deformation_dep = np.load(nodef_dep_savefile)
        self.border_mask = np.load(border_mask_savefile)

        # plt the reference images for checking
        # import matplotlib.pyplot as plt
        # fig, axs = plt.subplots(1, 3)
        # axs[0].imshow(self.no_deformation_gray, cmap='gray')
        # axs[1].imshow(self.no_deformation_dep, cmap='gray')
        # axs[2].imshow(self.border_mask, cmap='gray')
        # plt.show(block=True)
        # exit()

    def save_reference_images(self):

        # grab images for creating border from simulation
        no_deformation_rgb, no_deformation_dep, no_deformation_mask = self.get_imgs()
        no_deformation_gray = cv2.cvtColor(no_deformation_rgb.astype(np.float32), cv2.COLOR_BGR2GRAY)

        # convert mask from link/base ids to ones/zeros for border/not border
        mask_base_id = no_deformation_mask & ((1 << 24) - 1)
        mask_link_id = (no_deformation_mask >> 24) - 1
        border_mask = (mask_base_id == self.embodiment_id) & (mask_link_id == self.tactile_link_ids["body"]).astype(np.uint8)

        # create save file
        border_images_path = add_assets_path("reference_images")

        saved_file_dir = os.path.join(
            border_images_path,
            self.sensor_type,
            str(self.image_size[0]) + "x" + str(self.image_size[1]),
        )

        # create new directory
        os.makedirs(saved_file_dir, exist_ok=True)

        # save file names
        nodef_gray_savefile = os.path.join(saved_file_dir, "nodef_gray.npy")
        nodef_dep_savefile = os.path.join(saved_file_dir, "nodef_dep.npy")
        border_mask_savefile = os.path.join(saved_file_dir, "border_mask.npy")

        # save border images from simulation
        np.save(nodef_gray_savefile, no_deformation_gray)
        np.save(nodef_dep_savefile, no_deformation_dep)
        np.save(border_mask_savefile, border_mask)

        exit()

    def setup_camera_info(self):
        """
        Set parameters that define images from internal camera.
        """
        if self.sensor_type in ["standard_tactip", "right_angle_tactip", "flat_tactip", "right_angle_flat_tactip"]:
            self.focal_dist = 0.065
            self.fov = 60

        elif self.sensor_type in ["mini_tactip", "right_angle_mini_tactip"]:
            self.focal_dist = 0.065
            self.fov = 60

        elif self.sensor_type in ["standard_digit", "right_angle_digit", "forward_digit"]:
            self.focal_dist = 0.0015
            self.fov = 40
        elif self.sensor_type in ['standard_digitac', 'right_angle_digitac', 'forward_digitac']:
            self.focal_dist = 0.0015
            self.fov = 40

        self.pixel_width, self.pixel_height = self.image_size[0], self.image_size[1]
        self.aspect, self.nearplane, self.farplane = 1.0, 0.01, 1.0
        self.focal_length = 1.0 / (2 * np.tan((self.fov * (np.pi / 180)) / 2))  # not used but useful to know
        self.projection_matrix = self._pb.computeProjectionMatrixFOV(self.fov, self.aspect, self.nearplane, self.farplane)

    def update_cam_frame(self):

        # get the pose of the sensor body (where camera sits)
        sensor_body_pos, sensor_body_orn, _, _, _, _ = self._pb.getLinkState(
            self.embodiment_id, self.tactile_link_ids["body"], computeForwardKinematics=True
        )

        # set camera position relative to the sensor body
        if self.sensor_family == 'tactip':
            if self.sensor_type in ["standard", "mini_standard", "flat"]:
                cam_pos = (0, 0, 0.03)
                cam_rpy = (0, -np.pi / 2, np.pi)
            elif self.sensor_type in ["right_angle", "forward"]:
                cam_pos = (0, 0, 0.03)
                cam_rpy = (0, -np.pi / 2, 140 * np.pi / 180)
            elif self.sensor_type in ["mini_right_angle"]:
                cam_pos = (0, 0, 0.001)
                cam_rpy = (0, -np.pi / 2, 140 * np.pi / 180)

        elif self.sensor_family == 'digit':
            if self.sensor_type in ["standard"]:
                cam_pos = (-0.00095, .0139, 0.020)
                cam_rpy = (np.pi, -np.pi/2, np.pi/2)
            elif self.sensor_type in ["right_angle", "forward"]:
                cam_pos = (-0.00095, .0139, 0.005)
                cam_rpy = (np.pi, -np.pi/2, np.pi/2)
        elif self.sensor_family == 'digitac':
            if self.sensor_type in ["standard"]:
                cam_pos = (-0.00095, .0139, 0.020)
                cam_rpy = (np.pi, -np.pi/2, np.pi/2)
            elif self.sensor_type in ["right_angle", "forward"]:
                cam_pos = (-0.00095, .0139, 0.005)
                cam_rpy = (np.pi, -np.pi/2, np.pi/2)

        cam_orn = self._pb.getQuaternionFromEuler(cam_rpy)

        # get the camera frame relative to world frame
        self.camframe_pos, self.camframe_orn = self._pb.multiplyTransforms(sensor_body_pos, sensor_body_orn, cam_pos, cam_orn)

    def camframe_to_worldframe(self, pos, rpy):
        """
        Transforms a pose in camera frame to a pose in world frame.
        """
        pos = np.array(pos)
        rpy = np.array(rpy)
        orn = np.array(self._pb.getQuaternionFromEuler(rpy))

        worldframe_pos, worldframe_orn = self._pb.multiplyTransforms(self.camframe_pos, self.camframe_orn, pos, orn)
        worldframe_rpy = self._pb.getEulerFromQuaternion(worldframe_orn)

        return np.array(worldframe_pos), np.array(worldframe_rpy)

    def camvec_to_worldvec(self, camframe_vec):
        """
        Transforms a vector in work frame to a vector in world frame.
        """
        camframe_vec = np.array(camframe_vec)
        rot_matrix = np.array(self._pb.getMatrixFromQuaternion(self.camframe_orn)).reshape(3, 3)
        worldframe_vec = rot_matrix.dot(camframe_vec)

        return np.array(worldframe_vec)

    def get_imgs(self):
        """
        Pull some images from the synthetic camera
        """

        # update the camera frame
        self.update_cam_frame()

        # calculate view matrix
        foward_vector = self.camvec_to_worldvec([1, 0, 0])
        up_vector = self.camvec_to_worldvec([0, 0, 1])
        cam_target_pos = self.camframe_pos + self.focal_dist * np.array(foward_vector)

        view_matrix = self._pb.computeViewMatrix(
            self.camframe_pos,
            cam_target_pos,
            up_vector,
        )

        # draw a line at these points for debugging
        # extended_cam_pos = self.camframe_pos + np.array(foward_vector)
        # extended_up_pos  = self.camframe_pos + np.array(up_vector)
        # self._pb.addUserDebugLine(self.camframe_pos, extended_cam_pos, [0, 1, 1], parentObjectUniqueId=-1, parentLinkIndex=-1, lifeTime=0.1)
        # self._pb.addUserDebugLine(self.camframe_pos, extended_up_pos, [1, 0, 1], parentObjectUniqueId=-1, parentLinkIndex=-1, lifeTime=0.1)

        # projective texture runs faster but gives odd visuals
        flags = self._pb.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
        img_arr = self._pb.getCameraImage(
            self.pixel_width,
            self.pixel_height,
            view_matrix,
            self.projection_matrix,
            renderer=self._pb.ER_BULLET_HARDWARE_OPENGL,
            flags=flags,
        )

        # get images from returned array
        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgb = img_arr[2]  # color data RGB
        dep = img_arr[3]  # depth dataes
        mask = img_arr[4]  # mask dataes

        rgb = np.reshape(rgb, (h, w, 4))
        dep = np.reshape(dep, (h, w))
        mask = np.reshape(mask, (h, w))

        return rgb, dep, mask

    def sensor_camera(self):
        """
        Pull some images from the synthetic camera and manipulate them to become
        tacitle images.
        """

        # get the current images
        _, cur_dep, cur_mask = self.get_imgs()

        # get the difference between current images and undeformed counterparts
        diff_dep = np.subtract(cur_dep, self.no_deformation_dep)

        # remove noise from depth image
        eps = 1e-4
        diff_dep[(diff_dep >= -eps) & (diff_dep <= eps)] = 0

        # convert depth to penetration
        pen_img = np.abs(diff_dep)

        # convert dep to display format
        max_penetration = 0.05
        pen_img = ((np.clip(pen_img, 0, max_penetration) / max_penetration) * 255).astype(np.uint8)

        # reduce noise by setting all parts of the image where the sensor body is visible to zero
        mask_base_id = cur_mask & ((1 << 24) - 1)
        mask_link_id = (cur_mask >> 24) - 1
        full_mask = (mask_base_id == self.embodiment_id) & (mask_link_id == self.tactile_link_ids["body"])
        pen_img[full_mask] = 0

        # add border from ref image
        if not self.turn_off_border:
            pen_img[self.border_mask == 1] = self.no_deformation_gray[self.border_mask == 1]

        return pen_img

    def connect(self):
        """
        Setup plots if enabled.
        """
        # setup plot for rendering
        if self.show_tactile:
            cv2.namedWindow("tactile_window_{}".format(self.sensor_num), cv2.WINDOW_NORMAL)
            self._render_closed = False
        else:
            self._render_closed = True

    def reset(self):
        """
        Reset sensor
        """
        self.reset_tip()
        self.update_cam_frame()

    def reset_tip(self):
        """
        Reset the sensor core parameters here, could perform physics
        randomisations if required.
        """
        if self.sensor_core == "no_core":
            return None

        elif self.sensor_core == "fixed":
            # change dynamics
            self._pb.changeDynamics(
                self.embodiment_id,
                self.tactile_link_ids["tip"],
                contactDamping=self.sensor_dynamics["damping"],
                contactStiffness=self.sensor_dynamics["stiffness"],
            )
            self._pb.changeDynamics(
                self.embodiment_id, self.tactile_link_ids["tip"], lateralFriction=self.sensor_dynamics["friction"]
            )

    def process(self):
        """
        Return an image captured by the sensor.
        Also plot if enabled.
        """
        img = self.sensor_camera()
        # plot rendered image
        if not self._render_closed:
            cv2.imshow("tactile_window_{}".format(self.sensor_num), img)
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyWindow("tactile_window_{}".format(self.sensor_num))
                self._render_closed = True

        return img

    def get_observation(self):
        return self.process()

    def draw_camera_frame(self):
        rpy = [0, 0, 0]
        self._pb.addUserDebugLine(
            self.camframe_pos,
            self.camframe_to_worldframe([0.1, 0, 0], rpy)[0],
            [1, 0, 0],
            lifeTime=0.1,
        )
        self._pb.addUserDebugLine(
            self.camframe_pos,
            self.camframe_to_worldframe([0, 0.1, 0], rpy)[0],
            [0, 1, 0],
            lifeTime=0.1,
        )
        self._pb.addUserDebugLine(
            self.camframe_pos,
            self.camframe_to_worldframe([0, 0, 0.1], rpy)[0],
            [0, 0, 1],
            lifeTime=0.1,
        )

    def draw_sensor_frame(self):
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0.1, 0, 0],
            [1, 0, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tactile_link_ids["body"],
            lifeTime=0.1,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0.1, 0],
            [0, 1, 0],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tactile_link_ids["body"],
            lifeTime=0.1,
        )
        self._pb.addUserDebugLine(
            [0, 0, 0],
            [0, 0, 0.1],
            [0, 0, 1],
            parentObjectUniqueId=self.embodiment_id,
            parentLinkIndex=self.tactile_link_ids["body"],
            lifeTime=0.1,
        )