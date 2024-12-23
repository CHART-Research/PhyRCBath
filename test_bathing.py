import json
from pyrcareworld.envs.bathing_env import BathingEnv
import pyrcareworld.attributes.camera_attr as attr
import mediapipe as mp
import numpy as np
import cv2
import argparse
import math
import utils

scaling_fac = 5.546


class Pipeline:

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

    def __init__(self, use_graphics=False, dev=None):
        # Initialize the environment
        random_seed = utils.manikin_randomizer(True) # Comment for submission
        print(random_seed)

        self.env = BathingEnv(graphics=use_graphics, seed=random_seed) if dev == False else BathingEnv(graphics=use_graphics, executable_file="@editor")
        self.robot = self.env.get_robot()
        self.sponge = self.env.get_sponge()
        self.gripper = self.env.get_gripper()

        self.bathing_step = 3
        self.target_z = -0.4

        self.sky_cam = self.env.InstanceObject(name="Camera", id=123456,
            attr_type=attr.CameraAttr)
        self.manikin_cam = self.env.InstanceObject(name="Camera", id=223456,
            attr_type=attr.CameraAttr)
        self.sky_cam.SetTransform(position=[0, 3.8, 1.0], rotation=[90, 0, 0])
        self.manikin_cam.SetTransform(position=[0, 3.8, 0],
            rotation=[90, 0, 0])
        self.manikin_cam.GetDepth16Bit(1.0, 3.8)
        self.env.step(1)

        # Check for depth data
        if "depth" in self.manikin_cam.data:
            self.depth = np.frombuffer(self.manikin_cam.data["depth"], dtype=np.uint8)
            self.depth = cv2.imdecode(self.depth, cv2.IMREAD_GRAYSCALE)
        else:
            print("Warning: Depth data unavailable in manikin camera.")

        # self.depth = np.frombuffer(self.manikin_cam.data["depth"], dtype=np.uint8)
        # self.depth = cv2.imdecode(self.depth, cv2.IMREAD_GRAYSCALE)

    def get_sponge_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        sponge_details = self.env.get_sponge().data
        return sponge_details.get("position")

    def get_gripper_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        gripper = self.env.get_gripper()
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
        self.env.step()
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
        self.env.step(1)
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
        self.env.step()
        rgb = np.frombuffer(self.sky_cam.data["rgb"], dtype=np.uint8)
        # rgb2 = np.frombuffer(self.manikin_cam.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        # rgb2 = cv2.imdecode(rgb2, cv2.IMREAD_COLOR)

        # cv2.imwrite("skycam.png", rgb)
        # cv2.imwrite("manikincam.png", rgb2)

        self.env.step()

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
        self.env.step()
        rgb = np.frombuffer(self.manikin_cam.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        # Process the frame with Mediapipe
        results = self.pose.process(rgb)
        landmark_num = 0
        for id, landmark in enumerate(results.pose_landmarks.landmark):
            # print(f"landmark x: {landmark.x}, landmark y: {landmark.y}")
            # print(f"landmark y: {landmark.y}")
            # print("ID: ", id)
            
            # if id == 0: # nose
            #     self.target_x_nose = landmark.x
            #     self.target_y_nose = landmark.y
            # elif id == 11: # left shoulder
            #     self.target_x_left_sholder = landmark.x
            #     self.target_y_left_sholder = landmark.y
            # elif id == 12: # right shoulder
            #     self.target_x_right_sholder = landmark.x
            #     self.target_y_right_sholder = landmark.y
            # elif id == 24: # right hip
            #     self.target_x_right_hip = landmark.x
            #     self.target_y_right_hip = landmark.y
            # # elif id == 25:
            # #     self.target_x_25 = landmark.x

            # Get the dimensions of the frame
            h, w, _ = rgb.shape
            # convert normalised coordinates to pixels
            cx, cy = int(landmark.x * w), int(landmark.y * h)
            self.manikin_landmarks.append(
                    [landmark_num, self.mp_pose.PoseLandmark(id).name, cx, cy])
            landmark_num += 1
            # print(f"ID: {id}, Name: {self.mp_pose.PoseLandmark(id).name},
            #  X: {cx}, Y: {cy}")
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

    def compute_distance_3d(self, fixed_point, changing_point):
        """
        Compute the distance between two points in a 3D space.

        Parameters:
            fixed_point (list or tuple): Coordinates of the fixed point [x1, y1, z1].
            changing_point (list or tuple): Coordinates of the changing point [x2, y2, z2].
        
        Returns:
            float: Distance between the two points.
        """
        # Unpack coordinates
        x1, y1, z1 = fixed_point
        x2, y2, z2 = changing_point

        # Calculate Euclidean distance
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        return distance


    def movegripper(self, target_position) -> None:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        self.robot.IKTargetDoMove(
            position=target_position,
            duration=2,
            speed_based=False,
        )
        self.robot.WaitDo()
    
    def goto(self, target_position, target_rotation = None) -> None:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object
        """
        direction, rot, dist = utils.move(self.robot.data['position'], target_position, self.robot.data['rotation'][1])
        if direction == "Left" :
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight (rot, 1)


        # performing rotation action with required time step
        self.env.step(utils.calculate_step_rotation(rot)) 
        self.robot.MoveForward(dist, 1)
        self.env.step(utils.calculate_step_translation(dist))

        print("target_position: ", target_position)
        print("target_rotation: ", target_rotation)

        # if(target_rotation!= None):
        if(target_rotation!= "None"):
        # performing rotation action with required time step
            direction, rot  = utils.rotate(self.robot.data['rotation'][1], target_rotation)
            if direction == "Left" :
                self.robot.TurnLeft(rot, 1)
            else:
                self.robot.TurnRight (rot, 1)
            self.env.step(utils.calculate_step_rotation(rot))

    def get_manikin_coordinates_depth(self):
        # Get landmark pixel values
        self.generate_landmarks()
        self.target_nose = self.get_manikin(body_part="NOSE")

        self.target_right_shoulder = self.get_manikin(body_part="RIGHT_SHOULDER")
        self.target_left_shoulder = self.get_manikin(body_part="LEFT_SHOULDER")
        self.target_right_elbow = self.get_manikin(body_part="RIGHT_ELBOW")
        self.target_left_elbow = self.get_manikin(body_part="LEFT_ELBOW")
        self.target_chest = self.target_right_shoulder[0] - self.target_right_elbow[0]

        self.target_right_hip = self.get_manikin(body_part="RIGHT_HIP")
        self.target_left_wrist = self.get_manikin(body_part="LEFT_WRIST")
        self.target_right_wrist = self.get_manikin(body_part="RIGHT_WRIST")
        self.target_left_hip = self.get_manikin(body_part="LEFT_HIP")
        self.target_right_knee = self.get_manikin(body_part="RIGHT_KNEE")
        self.target_left_knee = self.get_manikin(body_part="LEFT_KNEE")
        self.target_right_ankle = self.get_manikin(body_part="RIGHT_ANKLE")
        self.target_left_ankle = self.get_manikin(body_part="LEFT_ANKLE")
        
        print("self.target_nose: ", self.target_nose)
        print("self.target_right_shoulder: ", self.target_right_shoulder)
        print("self.target_left_shoulder: ", self.target_left_shoulder)
        print("self.target_right_elbow: ", self.target_right_elbow)
        print("self.target_left_elbow: ", self.target_left_elbow)
        print("self.target_right_hip: ", self.target_right_hip)
        print("self.target_left_hip: ", self.target_left_hip)
        print("self.target_right_knee: ", self.target_right_knee)
        print("self.target_left_knee: ", self.target_left_knee)
        print("self.target_right_ankle: ", self.target_right_ankle)
        print("self.target_left_ankle: ", self.target_left_ankle)

    def dip_and_nav(self):
        # Load the waypoints data from a JSON file
        with open('waypoints.json') as f:
            self. waypoints_data = json.load(f)
            print(self. waypoints_data)  # Print the loaded waypoints data for debugging
        
        # Print the robot's current position and rotation
        print(self.robot.data['position'])
        print(self.robot.data['rotation'])


        # Randomize the robot and sponge poses for testing (comment this out for submission)
        self.robot = utils.random_robot_pose(self.robot)
        self.sponge = utils.random_sponge_pose(self.sponge)
        self.env.step(100)  # Wait for robot and sponge to stabilize

        # Lift the gripper to a safe height to avoid obstacles
        lift_gripper_position = [self.robot.data['position'][0], self.sponge.data['position'][1] + 0.15, self.robot.data['position'][2]]
        print(f"Raising gripper to safe height: {lift_gripper_position}")
        self.robot.IKTargetDoMove(
            position=lift_gripper_position,
            duration=2,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Load grasping position and rotation for the sponge from waypoints
        grasping_position = self. waypoints_data['grasping']['position']
        grasping_rotation = self. waypoints_data['grasping']['rotation']

        # Align grasping position with the sponge's current position
        grasping_position[0] = self.sponge.data['position'][0]

        # Calculate movement direction, required rotation, and distance to the sponge
        direction, rot, dist = utils.move(self.robot.data['position'], grasping_position, self.robot.data['rotation'][1])

        print("direction: ", direction)
        print("rot: ", rot)
        print("dist: ", dist)

        # Rotate the robot toward the sponge
        if direction == "Left":
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight(rot, 1)

        # Perform rotation step
        self.env.step(utils.calculate_step_rotation(rot))

        # Move forward to the sponge
        self.robot.MoveForward(dist, 1)
        self.env.step(utils.calculate_step_translation(dist))

        # Fine-tune robot's orientation to match the sponge's rotation
        direction, rot = utils.rotate(self.robot.data['rotation'][1], grasping_rotation)
        if direction == "Left":
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight(rot, 1)
        self.env.step(utils.calculate_step_rotation(rot))

        # Print the robot's updated position and rotation
        print(self.robot.data['position'])
        print(self.robot.data['rotation'])

        # Use IK to position the gripper directly above the sponge
        fine_tune_position = [self.sponge.data['position'][0], self.sponge.data['position'][1] + 0.2, self.sponge.data['position'][2]]
        print(f"Using IK to position above the sponge: {fine_tune_position}")
        self.robot.IKTargetDoMove(
            position=fine_tune_position,
            duration=2,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Lower the gripper to reach the sponge
        lower_position = [self.sponge.data['position'][0], self.sponge.data['position'][1] + 0.02, self.sponge.data['position'][2] + 0.05]
        print(f"Lowering gripper to reach sponge: {lower_position}")
        self.robot.IKTargetDoMove(
            position=lower_position,
            duration=1,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Close the gripper to grasp the sponge
        self.gripper.GripperClose()

        # Raise the gripper to a safe height
        lift_gripper_position = [self.robot.data['position'][0], self.sponge.data['position'][1] + 0.15, self.robot.data['position'][2]]
        print(f"Raising gripper to safe height: {lift_gripper_position}")
        self.robot.IKTargetDoMove(
            position=lift_gripper_position,
            duration=2,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Move backward to create space for dipping the sponge in water
        print(f"Move to water")
        self.robot.MoveBack(0.5, 1)
        self.env.step(utils.calculate_step_translation(0.5))

        # Position the robot above the water for dipping
        above_water_position = [-0.11, 0.8, 2.22]
        print(f"Move above water{above_water_position}")
        self.robot.IKTargetDoMove(
            position=above_water_position,
            duration=1,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Lower the gripper to dip the sponge in water
        water_position = [-0.11, 0.58, 2.22]
        print(f"Lowering gripper to dip water {water_position}")
        self.robot.IKTargetDoMove(
            position=water_position,
            duration=1,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Move the gripper back above the water
        print(f"Move above water{above_water_position}")
        self.robot.IKTargetDoMove(
            position=above_water_position,
            duration=1,
            speed_based=False,
        )

        # Raise the gripper back to a safe position
        lift_gripper_position = [self.robot.data['position'][0], 1.5, self.robot.data['position'][2]]
        print(f"Back to safe position: {lift_gripper_position}")
        self.robot.IKTargetDoMove(
            position=lift_gripper_position,
            duration=2,
            speed_based=False,
        )
        self.robot.WaitDo()

        # Move backward to exit the area
        self.robot.MoveBack(0.7, 1)
        self.env.step(utils.calculate_step_translation(0.7))

        self.get_manikin_coordinates_depth()

        # Move to the next waypoint (head of the bed)
        direction, rot, dist = utils.move(self.robot.data['position'], self.waypoints_data['bedhead']['position'], self.robot.data['rotation'][1])
        if direction == "Left":
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight(rot, 1)
        self.env.step(utils.calculate_step_rotation(rot))
        self.robot.MoveForward(dist, 1)
        self.env.step(utils.calculate_step_translation(dist))

        # Adjust orientation to match the bedhead rotation
        if self.waypoints_data['bedhead']['rotation'] != "None":
            direction, rot = utils.rotate(self.robot.data['rotation'][1], self. waypoints_data['bedhead']['rotation'])
            if direction == "Left":
                self.robot.TurnLeft(rot, 1)
            else:
                self.robot.TurnRight(rot, 1)
            self.env.step(utils.calculate_step_rotation(rot))

        # Move to the next waypoint (top left of the bed)
        direction, rot, dist = utils.move(self.robot.data['position'], self.waypoints_data['bedtl']['position'], self.robot.data['rotation'][1])
        if direction == "Left":
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight(rot, 1)
        self.env.step(utils.calculate_step_rotation(rot))
        self.robot.MoveForward(dist, 1)
        self.env.step(utils.calculate_step_translation(dist))

        # Adjust orientation to match the top-left rotation
        if self. waypoints_data['bedtl']['rotation'] != "None":
            direction, rot = utils.rotate(self.robot.data['rotation'][1], self. waypoints_data['bedtl']['rotation'])
            if direction == "Left":
                self.robot.TurnLeft(rot, 1)
            else:
                self.robot.TurnRight(rot, 1)
            self.env.step(utils.calculate_step_rotation(rot))



        ###### update target start position ###
        # self. waypoints_data['bathing1']['position'][0] = self.target_x_nose - 0.15
        self.waypoints_data['bathing1']['position'][0] = self.target_nose[0] - 0.195


        # Move to the bathing area waypoint
        direction, rot, dist = utils.move(self.robot.data['position'], self.waypoints_data['bathing1']['position'], self.robot.data['rotation'][1])
        if direction == "Left":
            self.robot.TurnLeft(rot, 1)
        else:
            self.robot.TurnRight(rot, 1)
        self.env.step(utils.calculate_step_rotation(rot))
        self.robot.MoveForward(dist, 1)
        self.env.step(utils.calculate_step_translation(dist))

        # Adjust orientation to match the bathing area rotation
        if self. waypoints_data['bathing1']['rotation'] != "None":
            direction, rot = utils.rotate(self.robot.data['rotation'][1], self. waypoints_data['bathing1']['rotation'])
            if direction == "Left":
                self.robot.TurnLeft(rot, 1)
            else:
                self.robot.TurnRight(rot, 1)
            self.env.step(utils.calculate_step_rotation(rot))
    


    def bathing_face(self):
        print("Bathing Face")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
            
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.16, self.target_nose[2]-0.01]
        self.robot.IKTargetDoMove(
            position=position,
            duration=3,
            speed_based=False,
        )
        self.env.step(200)

        # position = [sponge_position[0], sponge_position[1] - 0.16, self.target_nose[2]-0.1]
        # self.robot.IKTargetDoMove(
        #     position=position,
        #     duration=2,
        #     speed_based=False,
        # )
        # self.env.step(200)

        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] + 0.1, self.target_nose[2]-0.01]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)


    def bathing_shoulder_line(self):
        print("Bathing Shoulder Line")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")

        position = [sponge_position[0], sponge_position[1] - 0.1, sponge_position[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
        
        # start from right shoulder
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_nose[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # to left shoulder
        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_left_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Lift to safe position
        position = [sponge_position[0], sponge_position[1], self.target_left_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

    def bathing_shoulder_stamp(self):
        print("Bathing Shoulder Stamp")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
        
        # start from right shoulder
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_right_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_right_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # to left shoulder
        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1], self.target_left_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_left_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_left_shoulder[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

    def bathing_chest_line(self):
        print("Bathing Chest Line")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
        
        # start from right shoulder
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.19, self.target_nose[2]-0.01]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # to left shoulder
        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.19, self.target_left_shoulder[2]+0.09]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Lift to safe position
        position = [sponge_position[0], sponge_position[1], self.target_left_shoulder[2]+0.09]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

    def bathing_chest_stamp(self):
        print("Bathing Chest Stamp")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
        
        # start from right shoulder
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_right_shoulder[2]-0.05]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_right_shoulder[2]-0.05]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # to left shoulder
        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_left_shoulder[2]+0.05]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Lift to safe position
        position = [sponge_position[0], sponge_position[1], self.target_left_shoulder[2]+0.05]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

    def bathing_elbow_line(self):
        print("Bathing Elbow Line")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")

        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_nose[2]-0.015]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.18, self.target_left_elbow[2]+0.15]
        # print(f"Up: {position}")
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_left_elbow[2]+0.15]
        # print(f"Up: {position}")
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)
        
    def bathing_hip_line(self):
        print("Bathing Hip Line")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
            
        # position = [sponge_position[0], sponge_position[1] - 0.15, -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.17, self.target_nose[2]-0.015]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # position = [sponge_position[0], sponge_position[1], -0.1]
        position = [sponge_position[0], sponge_position[1] - 0.17, self.target_left_hip[2]+0.085]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_left_hip[2]+0.085]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

    def bathing_hip_knee(self):
        print("Bathing From Hip to Knee")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
            
        # Set on left hip
        print("Set on left hip")
        position = [sponge_position[0], sponge_position[1], self.target_left_hip[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_left_hip[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_left_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_right_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move backward 
        print(f"Move backward")
        self.robot.MoveBack(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

        print("Set on hip left for safe position")
        position = [sponge_position[0], sponge_position[1], self.target_left_hip[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

    def bathing_knee_ankle_left(self):
        print("Bathing From Left Knee to Left Ankle")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
            
        print("Set on left knee")
        position = [sponge_position[0], sponge_position[1], self.target_left_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_left_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_left_ankle[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_left_ankle[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move backward 
        print(f"Move backward")
        self.robot.MoveBack(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

    def bathing_knee_ankle_right(self):
        print("Bathing From Right Knee to Right Ankle")
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]
        # print(f"Sponge position: {sponge_position}")
        # print(f"Robot position: {robot_position}")
            
        print("Set on right knee")
        position = [sponge_position[0], sponge_position[1], self.target_right_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_right_knee[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

        position = [sponge_position[0], sponge_position[1] - 0.1, self.target_right_ankle[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        position = [sponge_position[0], sponge_position[1], self.target_right_ankle[2]]
        self.robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        # Move backward 
        print(f"Move backward")
        self.robot.MoveBack(1.0, 0.5)
        self.env.step(100)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)


    def bathing_horizontal_line_step(self):
        sponge_position = self.sponge.data["position"]
        robot_position = self.robot.data["position"]

        # Move the gripper to manikin position
        # start_manikin_position = [sponge_position[0], sponge_position[1] - 0.2 , -0.1]
        start_manikin_position = [sponge_position[0], sponge_position[1] - 0.2 , self.target_nose[2]]
        print(f"Move above manikin: {start_manikin_position}")
        self.robot.IKTargetDoMove(
            position=start_manikin_position,
            duration=2,
            speed_based=False,
        )
        self.env.step(200)

        self.bathing_face()

        #!!!!!!!! need a function to move robot to self.target_right_shoulder[0] coordinate
        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(0.5, 0.5)
        self.env.step(75)
        # self.robot.MoveForward(self.nose_sholder, 0.5)
        # self.env.step(int(50 * self.nose_sholder / 0.5))
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)
        # self.waypoints_data['bathing1']['position'][0] = self.target_left_shoulder[0]
        # self.goto(self.waypoints_data['bathing1']['position'], 270)
        self.bathing_shoulder_line()
        
        #!!!!!!!! need a function to move robot to self.target_right_elbow[0] coordinate
        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(0.5, 0.5)
        self.env.step(75)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)
        # self.waypoints_data['bathing1']['position'][0] = self.target_left_elbow[0]
        # self.goto(self.waypoints_data['bathing1']['position'], 270)
        self.bathing_chest_line()

        # self.robot.TurnRight(90, 1)
        # self.env.step(100)
        # self.robot.MoveForward(0.5, 0.5)
        # self.env.step(50)
        # self.robot.TurnLeft(90, 1)
        # self.env.step(100)

        #!!!!!!!! need a function to move robot to self.target_right_elbow[0] coordinate
        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(0.5, 0.5)
        self.env.step(75)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)
        # self.waypoints_data['bathing1']['position'][0] = self.target_left_elbow[0]
        # self.goto(self.waypoints_data['bathing1']['position'], 270)
        self.bathing_elbow_line()

        #!!!!!!!! need a function to move robot to self.target_right_hip[0] coordinate
        # Move forward 
        print(f"Move forward")
        self.robot.MoveForward(0.5, 0.5)
        self.env.step(75)
        # Stop
        print("Stop with TurnRight function")
        self.robot.TurnRight(0, 1)
        self.env.step(50)

        self.bathing_hip_line()


        # ###### lower body ######

        # self.bathing_hip_knee()

        # self.bathing_knee_ankle_left()

        # self.bathing_knee_ankle_right()

    def bathing(self):
        self.bathing_horizontal_line_step()


    def run(self):
        self.dip_and_nav()
        self.bathing()

def main():
    parser = argparse.ArgumentParser(description="Move robot from start to target position.")
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    parser.add_argument('-d', '--dev', action='store_true', help='Run in developer mode')
    args = parser.parse_args()

    pipeline = Pipeline(use_graphics=args.graphics, dev=args.dev)
    pipeline.run()

    # iteration = 20
    # score_table= {}
    # for i in range(iteration):
    #     pipeline = Pipeline(use_graphics=args.graphics, dev=args.dev)
    #     pipeline.run()
    #     with open('/home/koyo/.config/unity3d/RCareWorld/DressingPlayer/spongeScore.json') as f:
    #         score = json.load(f)
    #     score_table["trial" + str(i)]= score
    # # Convert into JSON
    # # File name is mydata.json
    # with open("score.json", "w") as final:
	#     json.dump(score_table, final)


    

if __name__ == "__main__":
    main()
