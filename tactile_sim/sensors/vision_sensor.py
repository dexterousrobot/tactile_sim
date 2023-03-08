import numpy as np
import cv2


def rgba_to_bgr(rgba):
    return cv2.cvtColor(rgba.astype(np.uint8), cv2.COLOR_RGBA2BGR)


def dep_to_bgr(dep):
    return cv2.cvtColor(np.clip((dep*255).astype(np.uint8), 0, 255), cv2.COLOR_GRAY2BGR)


def mask_to_bgr(mask):
    mask_to_colour = {
        0: (128, 0, 0),
        1: (0, 128, 0),
        2: (0, 0, 128),
        3: (128, 128, 0),
        4: (0, 128, 128),
        5: (128, 0, 128),
    }

    bgr = np.zeros(shape=(*mask.shape, 3), dtype=np.uint8)
    for key in np.unique(mask):
        colour_key = key % len(mask_to_colour)
        bgr[mask == key, ...] = mask_to_colour[colour_key]

    return bgr


class VisionSensor:
    def __init__(
        self,
        pb,
        image_size=[128, 128],
        dist=0.25,
        yaw=90.0,
        pitch=-25.0,
        pos=[0.6, 0.0, 0.0525],
        fov=75.0,
        near_val=0.1,
        far_val=100.0,
        show_vision=True,
        sensor_num=int(0),
    ):
        self._pb = pb
        self.image_size = image_size
        self.dist = dist
        self.yaw = yaw
        self.pitch = pitch
        self.pos = pos
        self.fov = fov
        self.near_val = near_val
        self.far_val = far_val
        self.show_vision = show_vision
        self.sensor_num = sensor_num

        self.setup_camera_info()
        self.connect()

    def setup_camera_info(self):
        """
        Set parameters that define images from internal camera.
        """
        # get an rgb image that matches the debug visualiser
        self.view_matrix = self._pb.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=self.pos,
            distance=self.dist,
            yaw=self.yaw,
            pitch=self.pitch,
            roll=0,
            upAxisIndex=2,
        )

        self.proj_matrix = self._pb.computeProjectionMatrixFOV(
            fov=self.fov,
            aspect=float(self.image_size[0]) / self.image_size[1],
            nearVal=self.near_val,
            farVal=self.far_val,
        )

    def get_imgs(self):
        """
        Pull some images from the synthetic camera
        """

        img_arr = self._pb.getCameraImage(
            self.image_size[0],
            self.image_size[1],
            self.view_matrix,
            self.proj_matrix,
            renderer=self._pb.ER_BULLET_HARDWARE_OPENGL,
        )

        # get images from returned array
        w = img_arr[0]  # width of the image, in pixels
        h = img_arr[1]  # height of the image, in pixels
        rgb = img_arr[2]  # color data RGB
        dep = img_arr[3]  # depth dataes
        mask = img_arr[4]  # mask dataes

        rgba = np.reshape(rgb, (h, w, 4))
        dep = np.reshape(dep, (h, w))
        mask = np.reshape(mask, (h, w))

        return rgba, dep, mask

    def connect(self):
        """
        Setup plots if enabled.
        """
        # setup plot for rendering
        if self.show_vision:
            cv2.namedWindow("vision_window_{}".format(self.sensor_num), cv2.WINDOW_NORMAL)
            self._render_closed = False
        else:
            self._render_closed = True

    def process(self):
        """
        Return an image captured by the sensor.
        Also plot if enabled.
        """
        rgba, dep, mask = self.get_imgs()

        # display rendered image
        if not self._render_closed:

            disp_bgr = rgba_to_bgr(rgba)
            disp_dep = dep_to_bgr(dep)
            disp_mask = mask_to_bgr(mask)
            disp_image = np.concatenate([disp_bgr, disp_dep, disp_mask], axis=1)

            cv2.imshow("vision_window_{}".format(self.sensor_num), disp_image)
            if cv2.waitKey(1) & 0xFF == 27:
                cv2.destroyWindow("tactile_window_{}".format(self.sensor_num))
                self._render_closed = True

        return rgba, dep, mask

    def get_observation(self):
        return self.process()
