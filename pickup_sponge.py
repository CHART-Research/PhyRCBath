import json
from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
import cv2
import argparse

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
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")

    # First, raise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot_position[0], sponge_position[1] + 0.15, robot_position[2]]
    print(f"Raising gripper to safe height: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(300)

    robot.TurnLeft(85, 1)
    env.step(300)

    #Position to pick the sponge
    print(f"Move to pick sponge")
    robot.MoveForward(0.25, 0.2)
    env.step(300)

    # Use IK to position the gripper directly above the sponge (adjusting Z direction)
    fine_tune_position = [sponge_position[0], sponge_position[1] + 0.2, sponge_position[2]]  # Position above the sponge at safe height
    print(f"Using IK to position above the sponge: {fine_tune_position}")
    robot.IKTargetDoMove(
        position=fine_tune_position,
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(100)

    # Lower the gripper to reach the sponge
    lower_position = [sponge_position[0], sponge_position[1]+0.05, sponge_position[2]]
    print(f"Lowering gripper to reach sponge: {lower_position}")
    robot.IKTargetDoMove(
        position=lower_position,  # Move directly above the sponge to grasp it
        duration=1,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(100)

    # Control the gripper to grasp the sponge
    gripper = env.get_gripper()
    gripper.GripperClose()
    env.step(300)


    # Rise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot_position[0], sponge_position[1] + 0.15, robot_position[2]]
    print(f"Raising gripper to safe height: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)

    #Move to dip water
    print(f"Move to water")
    robot.MoveBack(0.25, 0.2)
    env.step(300)

    print(f"Sponge position: {sponge_position}")

    # Move the robot above the water
    above_water_position = [-0.11, 0.8, 2.22]
    print(f"Move above water{above_water_position}")
    robot.IKTargetDoMove(
        position=above_water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)

    # Lower the gripper to dip the water
    #water_position = [-0.10899999737739563, 0.5829999923706055, 2.2269999980926514]
    water_position = [-0.11, 0.58, 2.22]
    print(f"Lowering gripper to dip water {water_position}")
    robot.IKTargetDoMove(
        position=water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)


    # Move above water
    print(f"Move above water{above_water_position}")
    robot.IKTargetDoMove(
        position=above_water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)

    # Rise the gripper back to a safe position
    lift_gripper_position = [robot_position[0], 0.8, robot_position[2]]
    print(f"Back to safe position: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)

    #Move out 
    robot.MoveBack(0.5, 0.2)
    env.step(300)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
