import json
from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
import cv2
import argparse
import math

scaling_fac = 5.546

def initialize_environment(use_graphics=False):
    """Initialize the BathingEnv environment and return robot and sponge."""
    env = BathingEnv(graphics=use_graphics)
    robot = env.get_robot()
    sponge = env.get_sponge()
    return env, robot, sponge

def compute_distance_3d(fixed_point, changing_point):
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

def movement_from_A_to_landmark(env, robot, fixed_point, final_point):
    """
    Move the robot from a fixed point to a final point.

    Parameters:
        env (BathingEnv): The simulation environment.
        robot: The robot object.
        fixed_point (list or tuple): Starting coordinates [x1, y1, z1].
        final_point (list or tuple): Target coordinates [x2, y2, z2].
    """
    mid_x = 1.9
    x1, y1, z1 = fixed_point
    trans_point = (mid_x, y1, z1)
    x2, y2, z2 = final_point

    # Move to transitional point
    distance = compute_distance_3d(fixed_point, trans_point)
    print(f"Move back by {distance * scaling_fac} units")
    robot.MoveBack(distance * scaling_fac, 0.5)
    env.step(500)

    # Turn towards the final point
    if z2 < z1:
        print("Turn left")
        robot.TurnRight(90, 1)
    else:
        print("Turn right")
        robot.TurnLeft(90, 1)
    env.step(300)

    # Move to transitional point
    robot_position = robot.data["position"]
    trans_point=(mid_x,y2,z2)
    distance = compute_distance_3d(robot_position, trans_point)
    print(f"Move forward by {distance * scaling_fac} units")
    robot.MoveForward(distance * scaling_fac, 0.5)
    env.step(700)

    # Turn towards the final position
    if x2 < mid_x:
        print("Turn left")
        robot.TurnRight(90, 1)
    else:
        print("Turn right")
        robot.TurnLeft(90, 1)
    env.step(300)

    # Move to final point
    robot_position = robot.data["position"]
    distance = compute_distance_3d(robot_position, final_point)
    print(f"Move forward by {distance * scaling_fac} units")
    robot.MoveForward(distance * scaling_fac, 0.5)
    env.step(400)
    # env.step(200)

    # Stop
    print("Stop with TurnRight function")
    robot.TurnRight(0, 1)
    env.step(200)

    print(f"Reached position: {final_point}")

def bathing_line_step(env, robot, sponge, num_bathing_step_per_line):
    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")

    # Move the gripper to manikin position
    # bed_center = 1.3711309432983398 # This value is used with sponge_position so it should not be from target_position
    bed_center = 0.0
    bed_center_delta = input("Enter displacement from bed center (float), Left(+) or Right(-) (width of half bed is around 0.5?): ") 
    start_manikin_position = [sponge_position[0], sponge_position[1], bed_center + float(bed_center_delta)]
    print(f"Move above manikin: {start_manikin_position}")
    robot.IKTargetDoMove(
        position=start_manikin_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)

    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")

    for i in range(num_bathing_step_per_line):
        sponge_position_delta = input("Enter displacement of sponge heigh (float), Up(+) or Down(-) (0.1 is too much sometimes): ")
        position = [sponge_position[0], sponge_position[1] + float(sponge_position_delta), sponge_position[2]]
        # print(f"Down: {position}")
        robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        env.step(300)

        sponge_position_delta = input("Enter displacement of sponge heigh (float), Up(+) or Down(-) (0.1 is too much sometimes): ")
        position = [sponge_position[0], sponge_position[1] + float(sponge_position_delta), sponge_position[2]]
        # print(f"Up: {position}")
        robot.IKTargetDoMove(
            position=position,
            duration=2,
            speed_based=False,
        )
        env.step(300)

    # Move forward 
    print(f"Move forward")
    robot.MoveForward(0.3, 0.5)
    # env.step(400)
    env.step(200)

    # # Stop
    # print("Stop with TurnRight function")
    # robot.TurnRight(0, 1)
    # env.step(300)


def main(use_graphics=False):
    # Initialize environment and robot
    env, robot, sponge = initialize_environment(use_graphics)
    sponge = env.get_sponge()
    gripper=env.get_gripper()
    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    joint_position = robot.data["joint_positions"]
    gripper_position=gripper.data["position"]
    num_joints = robot.data["number_of_moveable_joints"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")
    print(f"Robot joint position: {joint_position}")
    print(f"Gripper position: {gripper_position}")
    print(f"Number of moveable joints: {num_joints}")
    print(f"Robot Keys: {robot.data.keys()}")
    joint_stiffness = robot.data["joint_stiffness"]
    joint_damping = robot.data["joint_damping"]
    print(f"Joint stiffness: {joint_stiffness}")
    print(f"Joint damping: {joint_damping}")
    # robot.SetIndexJointPosition(0, -0.2)
    # breakpoint()
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
    # Turn left
    print(f"Turn left")
    # robot.TurnLeft(85, 1)
    robot.TurnLeft(90, 1)
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
    env.step(200)
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
    env.step(200)
    # Lower the gripper to dip the water
    #water_position = [-0.10899999737739563, 0.5829999923706055, 2.2269999980926514]
    water_position = [-0.11, 0.58, 2.22]
    print(f"Lowering gripper to dip water {water_position}")
    robot.IKTargetDoMove(
        position=water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(200)

    # Move above water
    print(f"Move above water{above_water_position}")
    robot.IKTargetDoMove(
        position=above_water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(200)
    # Rise the gripper back to a safe position
    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")
    # lift_gripper_position = [robot_position[0], 0.8, robot_position[2]]
    # lift_gripper_position = [robot_position[0], 1.0, robot_position[2]]
    # lift_gripper_position = [sponge_position[0] - 0.03, sponge_position[1] + 0.22, sponge_position[2] - 0.9]
    lift_gripper_position = [sponge_position[0] - 0.03, sponge_position[1] + 0.3, sponge_position[2] - 0.9]
    print(f"Back to safe position: {lift_gripper_position}")
    # breakpoint()
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    robot_position_prev = robot.data["position"]
    print(f"Robot position: {robot_position_prev}")
    env.step(200)
    # Define start and target positions
    start_position = robot.data["position"]
    # target_position = [0.29251301288604736, 0.2508874237537384, 1.3711309432983398+0.5]
    # target_position = [0.29251301288604736, 0.2508874237537384, 1.3711309432983398+0.4]
    # !!! The base is not in parallel with bed frame
    target_position = [0.29251301288604736, 0.2508874237537384, 1.3711309432983398+0.225]
    # target_position = [0.29251301288604736, 0.2508874237537384, 1.3711309432983398+0.2] # The position is different with the same code

    # Move the robot to the target position
    movement_from_A_to_landmark(env, robot, start_position, target_position)


    sponge_position = sponge.data["position"]
    robot_position = robot.data["position"]
    print(f"Sponge position: {sponge_position}")
    print(f"Robot position: {robot_position}")

    bathing_step = 5

    for i in range(bathing_step):
        num_bathing_step_per_line = input("Enter the number of bathing in this horizontal line (int): ")
        bathing_line_step(env, robot, sponge, int(num_bathing_step_per_line))

    
    breakpoint()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Move robot from start to target position.")
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    main(use_graphics=args.graphics)
