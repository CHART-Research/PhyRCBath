from pyrcareworld.envs.bathing_env import BathingEnv
import pyrcareworld.attributes.camera_attr as attr
import numpy as np
import cv2
import mediapipe as mp


class Perception:

    # class variables
    img_size = 512
    world_x_min = -2
    world_x_max = 2
    world_range_x = world_x_max - world_x_min
    world_z_min = -1
    world_z_max = 3
    world_range_z = world_z_max - world_z_min
    camera_min = 1
    camera_max = img_size
    camera_range = camera_max - camera_min
    sky_cam = None
    manikin_cam = None
    manikin_landmarks = []
    depth = None

    # Initialize Mediapipe Pose
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    pose = mp_pose.Pose(min_detection_confidence=0.5,
        min_tracking_confidence=0.5)

    def __init__(self, env: BathingEnv):
        self.environment = env
        self.sky_cam = env.InstanceObject(name="Camera", id=123456,
            attr_type=attr.CameraAttr)
        self.manikin_cam = env.InstanceObject(name="Camera", id=223456,
            attr_type=attr.CameraAttr)
        self.sky_cam.SetTransform(position=[0, 3.8, 1.0], rotation=[90, 0, 0])
        self.manikin_cam.SetTransform(position=[0, 3.8, 0],
            rotation=[90, 0, 0])
        self.manikin_cam.GetDepth16Bit(1.0, 3.8)
        self.environment.step(1)

        print("Manikin Camera Data Keys:", self.manikin_cam.data.keys())


        self.depth = np.frombuffer(self.manikin_cam.data["depth"], dtype=np.uint8)
        self.depth = cv2.imdecode(self.depth, cv2.IMREAD_GRAYSCALE)

    def get_sponge_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        sponge_details = self.environment.get_sponge().data
        return sponge_details.get("position")

    def get_gripper_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        gripper = self.environment.get_gripper()
        return gripper.data['position']

    def camera_to_world(self, camera_x, camera_z) -> list:
        """
        Get the global coordinates in the world from the (x, y)
        pixel coordinate in an image
        :param camera_x: the x-coordinate pixel (horizontal)
        :param camera_z: the y-coordinate pixel (vertical)
        :return: a list of the (x, y, z) world coordinates of the coordinate
        """
        world_x = (((camera_x - self.camera_min) * self.world_range_x) /
            (self.camera_range - 1)) + self.world_x_min
        world_z = ((((self.img_size - camera_z) - self.camera_min) *
            self.world_range_z)/ (self.camera_range - 1)) + self.world_z_min
        return ([world_x, 0.0, world_z])

    def manikincam_to_world(self, camera_x, camera_z) -> list:
        """
        Get the global coordinates in the world from the (x, y) pixel 
        coordinate in the manikin cam image
        :param camera_x: the x-coordinate pixel (horizontal)
        :param camera_z: the y-coordinate pixel (vertical)
        :return: a list of the (x, y, z) world coordinates of the coordinate
        """
        world_x = (((camera_x - self.camera_min) * 4) /
            (self.camera_range - 1)) + (-2)
        world_z = ((((self.img_size - camera_z) - self.camera_min) * 4) /
            (self.camera_range - 1)) + -2
        return ([world_x, 0.0, world_z])

    def take_pictures(self):
        """
        Debug method to take images from both cameras
        """
        self.sky_cam.GetRGB(512, 512)
        self.manikin_cam.GetRGB(512, 512)
        self.environment.step()
        rgb = np.frombuffer(self.sky_cam.data["rgb"], dtype=np.uint8)
        rgb2 = np.frombuffer(self.manikin_cam.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        rgb2 = cv2.imdecode(rgb2, cv2.IMREAD_COLOR)

        cv2.imwrite("skycam_1.png", rgb)
        cv2.imwrite("manikincam_1.png", rgb2)
    
    def get_depth(self, landmark_num: int = None, body_part: str = None):
        """
        Get the depth in metres of a manikin landmark
        :param landmark_num: the number of the landmark
        :param body_part: the name of the landmark
        :return: a distance in metres from the floor
        """
        self.manikin_cam.GetDepth16Bit(1.0, 3.8)
        self.environment.step(1)
        self.depth = np.frombuffer(self.manikin_cam.data["depth"], dtype=np.uint8)
        self.depth = cv2.imdecode(self.depth, cv2.IMREAD_GRAYSCALE)
        for landmark in self.manikin_landmarks:
            if landmark[0] == landmark_num or landmark[1] == body_part:
                print(landmark)
                depth_pixel_value = self.depth[landmark[2],landmark[3]]
                print("depth value: ", depth_pixel_value)
                if (depth_pixel_value == 255):
                    scaled_depth = 3.8
                else:
                    scaled_depth = (depth_pixel_value / 255) * (3.8 - 1.0)
                print("distance: ", scaled_depth)
                return 3.8 - scaled_depth
    
    def get_water_tank(self) -> list:
        """
        Get the global coordinates of the water tank in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coords of the centre of the water tank
        """
        self.sky_cam.GetRGB(512, 512)
        # self.manikin_cam.GetRGB(512, 512)
        self.environment.step()
        rgb = np.frombuffer(self.sky_cam.data["rgb"], dtype=np.uint8)
        # rgb2 = np.frombuffer(self.manikin_cam.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        # rgb2 = cv2.imdecode(rgb2, cv2.IMREAD_COLOR)

        # cv2.imwrite("skycam.png", rgb)
        # cv2.imwrite("manikincam.png", rgb2)

        self.environment.step()

        # Convert to graycsale
        img_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        # Detect blobs.
        keypoints = detector.detect(img_gray)

        bowl_x = 0
        bowl_z = 0
        size = 0
        for k in keypoints:
            if k.size > size:
                bowl_x = k.pt[0]
                bowl_z = k.pt[1]
                size = k.size

        return self.camera_to_world(bowl_x, bowl_z)
    
    def generate_landmarks(self):
        """Generate the landmarks of the manikin using MediaPipe"""
        self.manikin_cam.GetRGB(512, 512)
        self.environment.step()
        rgb = np.frombuffer(self.manikin_cam.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        # Process the frame with Mediapipe
        results = self.pose.process(rgb)
        landmark_num = 0
        for id, landmark in enumerate(results.pose_landmarks.landmark):
            # Get the dimensions of the frame
            h, w, _ = rgb.shape
            # convert normalised coordinates to pixels
            cx, cy = int(landmark.x * w), int(landmark.y * h)
            self.manikin_landmarks.append(
                    [landmark_num, self.mp_pose.PoseLandmark(id).name, cx, cy])
            landmark_num += 1
            # print(f"ID: {id}, Name: {self.mp_pose.PoseLandmark(id).name},
            # X: {cx}, Y: {cy}")
            cv2.circle(rgb, (int(cx), int(cy)), 5, (255,0,0), cv2.FILLED)
            cv2.imwrite('skelly_new.png', rgb)

    def get_manikin(self, body_part: str = None, landmark_num: int = None) -> list:
        """
        Get the global coordinates of the manikin's specified body part
        :param body_part: string name of the body part
        :return: a list of the (x, y, z) coordinates of the body part
        """
        for landmark in self.manikin_landmarks:
            if landmark[0] == landmark_num or landmark[1] == body_part:
                return self.manikincam_to_world(landmark[2], landmark[3])
