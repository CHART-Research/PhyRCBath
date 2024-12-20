import json
from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
import cv2
import argparse
import time

def _main(use_graphics=False):
    if use_graphics:
        text = """
        An example of the usage of the bathing environment.

        The sponge will be attached to the robot's hand if the grasp center and the sponge are close enough. (distance < 0.1m)  
        The sponge will be detached from the robot's hand if you call the GripperOpen() function.

        You can obtain low level information of the sponge, the robot, and use unlimited numbers of cameras to observe the scene.

        The threshold for a comfortable force on the human body is set to 1-6N.

        Check the website detailed rubric. After each run of the simulation, a json file will be generated in the current directory (~/.config/unity3d/RCareWorld/BathingPlayer).

        The path may be different according to the OS and your computer configuration.
        """

        #print(text) 
    # Initialize the environment
    env = BathingEnv(graphics=use_graphics)
    #print(f"env.attrs: {env.attrs}")

    robot = env.get_robot()
    env.step()
    #print(f"robot data: {robot.data}")
    
    # Obtain sponge data to navigate towards it
    sponge = env.get_sponge()

    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    joint_position = robot.data["joint_positions"]
    num_joints = robot.data["number_of_moveable_joints"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")
    print(f"Robot joint position: {joint_position}")
    print(f"Number of moveable joints: {num_joints}")
    print(f"Robot Keys: {robot.data.keys()}")

    joint_stiffness = robot.data["joint_stiffness"]
    joint_damping = robot.data["joint_damping"]
    print(f"Joint stiffness: {joint_stiffness}")
    print(f"Joint damping: {joint_damping}")




    # # Turn right
    # print(f"Turn right")
    # robot.TurnRight(90, 1)
    # env.step(300)

    # # Move forward 
    # print(f"Move forward")
    # robot.MoveForward(4.0, 0.5) # Move a robot 4.0 mm with 0.5 mm/s -> It takes 8 seconds
    # print("start")
    # # Record the start time
    # start_time = time.time()
    # env.step(400)
    # end_time = time.time()
    # # Calculate the elapsed time in seconds
    # elapsed_time_minutes = (end_time - start_time)
    # print(f"It took {elapsed_time_minutes} seconds") # It took 8.017067432403564 seconds
    # # -----> env.step(100) roughly corresponds to 2.0 seconds

    # breakpoint()


    # Turn left to cut the manipulation part
    print(f"Turn left for test")
    robot.TurnLeft(90, 1)
    env.step(300)

    # # Move back 
    # print(f"Move back")
    # robot.MoveBack(1.0, 0.2) # It takes 10 seconds
    # start_time = time.time()
    # env.step(500)
    # end_time = time.time()
    # # Calculate the elapsed time in seconds
    # elapsed_time_minutes = (end_time - start_time)
    # print(f"It took {elapsed_time_minutes} seconds")

    # # Move back 
    # print(f"Move back")
    # robot.MoveBack(1.0, 0.4) # It takes 5 seconds
    # start_time = time.time()
    # env.step(250)
    # end_time = time.time()
    # # Calculate the elapsed time in seconds
    # elapsed_time_minutes = (end_time - start_time)
    # print(f"It took {elapsed_time_minutes} seconds")

    # Move back 
    print(f"Move back")
    robot.MoveBack(1.0, 0.4) # It takes 5 seconds
    start_time = time.time()
    env.step(250)
    end_time = time.time()
    # Calculate the elapsed time in seconds
    elapsed_time_minutes = (end_time - start_time)
    print(f"It took {elapsed_time_minutes} seconds")

    # # Turn left
    # print(f"Turn left")
    # robot.TurnLeft(90, 1)
    # env.step(300)

    # # Move forward 
    # print(f"Move forward")
    # # robot.MoveForward(11.0, 0.5)
    # robot.MoveForward(10.0, 0.5) # It takes 20.0 seconds
    # env.step(1000)

    # # Turn right
    # print(f"Turn right")
    # robot.TurnRight(90, 1)
    # env.step(300)

    # # Move forward 
    # print(f"Move forward")
    # # robot.MoveForward(1.5, 0.3)
    # robot.MoveForward(4.0, 0.5)
    # env.step(400)

    # # Stop
    # print(f"Use TurnRight function to stop")
    # robot.TurnRight(0, 1)
    # env.step(300)


    breakpoint()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
