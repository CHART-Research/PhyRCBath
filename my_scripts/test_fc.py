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
    num_joints = robot.data["number_of_moveable_joints"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")
    print(f"Number of moveable joints: {num_joints}")
    print(robot.data.keys())

    joint_stiffness = robot.data["joint_stiffness"]
    joint_damping = robot.data["joint_damping"]
    print(f"Joint stiffness: {joint_stiffness}")
    print(f"Joint damping: {joint_damping}")
    


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

    stiffness = 100.0
    damping = 100.0
    print(f"Set joint stiffness {stiffness}")
    print(f"Set joint damping {damping}")
    robot.SetJointStiffness([stiffness] * 12 + [0.0, 0.0])
    robot.SetJointDamping([damping] * 12 + [1000.0, 1000.0])
    env.step(300)

    # # Dynamic adjustment for specific tasks
    # for i in range(14):
    #     if i < 12:  # Precision joints
    #         robot.SetJointStiffnessAtIndex(i, 80000.0)
    #         robot.SetJointDampingAtIndex(i, 7000.0)
    #     else:  # Compliant joints
    #         robot.SetJointStiffnessAtIndex(i, 0.0)
    #         robot.SetJointDampingAtIndex(i, 1000.0)
    

    # # Turn left to cut the manipulation part
    # print(f"Turn left for test")
    # robot.TurnLeft(90, 1)
    # env.step(300)

    # Move back 
    print(f"Move back")
    robot.MoveBack(7.0, 0.5)
    env.step(500)

    # Turn left
    print(f"Turn left")
    robot.TurnLeft(90, 1)
    env.step(300)

    # Move forward 
    print(f"Move forward")
    robot.MoveForward(11.0, 0.5)
    env.step(800)

    # Turn right
    print(f"Turn right")
    robot.TurnRight(90, 1)
    env.step(300)

    # Move forward 
    print(f"Move forward")
    robot.MoveForward(3.0, 0.3)
    env.step(200)

    # # Initial values for distance and speed
    # initial_distance = 1.5
    # initial_speed = 0.3
    # steps = 10  # Number of steps for gradual reduction

    # # Calculate the decrement for each step
    # distance_decrement = initial_distance / steps
    # speed_decrement = initial_speed / steps

    # # Gradually reduce distance and speed
    # for i in range(steps):
    #     current_distance = initial_distance - (i * distance_decrement)
    #     current_speed = initial_speed - (i * speed_decrement)
        
    #     # Ensure values don't go below zero
    #     current_distance = max(0, current_distance)
    #     current_speed = max(0, current_speed)
        
    #     print(f"Move forward with distance: {current_distance} and speed: {current_speed}")
    #     robot.MoveForward(current_distance, current_speed)
    #     env.step(200)

    #     # Stop if both distance and speed reach zero
    #     if current_distance == 0 and current_speed == 0:
    #         break


    # Stop
    print("Stop")
    robot.StopMovement()
    env.step(300)

    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")

    # Move the gripper to manikin position
    start_manikin_position = [robot_position[0], sponge_position[1] + 0.3, robot_position[2]]
    print(f"Move above manikin: {start_manikin_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)


    time_joint_positions = [
    [0.1, 0.2, 0.3],  # Position at t=1
    [0.15, 0.25, 0.35],  # Position at t=2
    ]
    interval = 100  # Interval in ms
    robot.SetJointPositionContinue(interval, time_joint_positions)




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
