import json
import utils
from pyrcareworld.envs.bathing_env import BathingEnv
import bathing_perception as perception
import navigation
import numpy as np
import math
import cv2
import argparse



def _main(use_graphics=False, dev=None):
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

        print(text) 
    # Initialize the environment
    random_seed = utils.manikin_randomizer(True) # Comment for submission
    #print(random_seed)
    env = BathingEnv(graphics=use_graphics, seed=random_seed) if dev == False else BathingEnv(graphics=use_graphics, executable_file="@editor")
    robot = env.get_robot()
    sponge = env.get_sponge()
 
    p = perception.Perception(env)
    nav = navigation.Navigation(env, robot)
    env.step(100) # wait for cameras to initialise
    p.take_pictures()
    p.generate_landmarks() # generate mediapipe landmarks
    print(p.manikin_landmarks)
    nose = p.get_manikin(body_part="NOSE")
    l_eye = p.get_manikin(body_part="LEFT_EYE")
    foot =  p.get_manikin(body_part="RIGHT_ANKLE")
    lfoot =  p.get_manikin(body_part="LEFT_ANKLE")
    depth =  p.get_depth (0, "NOSE")
    depth2 =  p.get_depth (2, "LEFT_EYE")
    depth3 =  p.get_depth (32, "RIGHT_ANKLE")
    depth4 =  p.get_depth (117, "LEFT_ANKLE")
    print("depth: ", depth)
    print("nose: ", nose)
    print("depth2: ", depth2)
    print("l_eye: ", l_eye)
    print("depth3: ", depth3)
    print("Rfoot: ", foot)
    print("depth4: ", depth4)
    print("lfoot: ", lfoot)
    p.take_pictures()
    #env.Pend()
    depth =  p.get_depth (0, "NOSE")
    print("depth: ", depth)
    #env.Pend()
    # read waypoints from json file
    with open('waypoints.json') as f:
        waypoints_data = json.load(f)
        print(waypoints_data)
    
    print(robot.data['position'])
    print(robot.data['rotation'])


    #env.Pend()
    # random robot pose and sponge. Comment this section for submission! 
    #robot = utils.random_robot_pose(robot)
    #sponge = utils.random_sponge_pose(sponge)
    #env.step(100) #waiting for robot and sponge to stabilise
    #----------------------------------------------#
    # env.Pend()
    # First, raise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot.data['position'][0], sponge.data['position'][1]+ 0.15, robot.data['position'][2]]
    print(f"Raising gripper to safe height: {lift_gripper_position}")
    nav.movegripper(lift_gripper_position)
    #env.Pend()

    # loading grasping position for sponge
    grasping_position = waypoints_data['grasping']['position']
    grasping_rotation = waypoints_data['grasping']['rotation']
    nav.goto(grasping_position, grasping_rotation)


    print(robot.data['position'])
    print(robot.data['rotation'])



#Position to pick the sponge
    # print(f"Move to pick sponge")
    # robot.MoveForward(0.25, 0.2)
    # env.step(300)

    # Use IK to position the gripper directly above the sponge (adjusting Z direction)
    fine_tune_position = [sponge.data['position'][0], sponge.data['position'][1] + 0.2, sponge.data['position'][2]]  # Position above the sponge at safe height
    print(f"Using IK to position above the sponge: {fine_tune_position}")
    nav.movegripper(fine_tune_position)

    # Lower the gripper to reach the sponge
    lower_position = [sponge.data['position'][0], sponge.data['position'][1]+0.02, sponge.data['position'][2] + 0.05]
    print(f"Lowering gripper to reach sponge: {lower_position}")
    nav.movegripper(lower_position)
    # Control the gripper to grasp the sponge
    gripper = env.get_gripper()
    gripper.GripperClose()
    #env.step(300)


    # Rise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot.data['position'][0], sponge.data['position'][1] + 0.15, robot.data['position'][2]]
    nav.movegripper(lift_gripper_position)
   
    #Move to dip water
    print(f"Move to water")
    robot.MoveBack(0.4, 1)
    env.step(utils.calculate_step_translation(0.5))

    #print(f"Sponge position: {sponge_position}")

    # Move the robot above the water
    above_water_position = [-0.11, 0.8, 2.22]
    print(f"Move above water{above_water_position}")
    nav.movegripper(above_water_position)
    #env.step(300)

    # Lower the gripper to dip the water
    #water_position = [-0.10899999737739563, 0.5829999923706055, 2.2269999980926514]
    water_position = [-0.11, 0.58, 2.22]
    print(f"Lowering gripper to dip water {water_position}")
    nav.movegripper(water_position)
    #env.step(300)


    # Move above water
    print(f"Move above water{above_water_position}")
    nav.movegripper(above_water_position)
    #env.step(300)

    # Rise the gripper back to a safe position
    lift_gripper_position = [robot.data['position'][0], 1.5, robot.data['position'][2]]
    print(f"Back to safe position: {lift_gripper_position}")
    nav.movegripper(lift_gripper_position)
    #env.step(300)

    #Decide which side of the bed is the most appropriate for bathing based on the manikin CoG



    #Move out 
    robot.MoveBack(0.7, 1)
    env.step(utils.calculate_step_translation(0.7))

    #Move to head
    nav.goto(waypoints_data['bedhead']['position'], waypoints_data['bedhead']['rotation'])
    
    #Move to tl 
    nav.goto(waypoints_data['bedtl']['position'], waypoints_data['bedtl']['rotation'])
     #env.Pend()

    #Move to bathing area
    nose = p.get_manikin(body_part="NOSE")
    l_eye = p.get_manikin(body_part="LEFT_EYE")
    depth =  p.get_depth (0, "NOSE")
    depth2 =  p.get_depth (2, "LEFT_EYE")
    bathing_start = waypoints_data['bathing1']['position']
    print("depth: ", depth)
    print("nose: ", nose)
    print("depth2: ", depth2)
    print("l_eye: ", l_eye)

    env.Pend()
    #bathing_start[0] =  nose[0]  

    direction, rot, dist = utils.move(robot.data['position'], bathing_start, robot.data['rotation'][1])
    if direction == "Left" :
        robot.TurnLeft(rot, 1)
    else:
        robot.TurnRight (rot, 1)

    # performing rotation action with required time step
    env.step(utils.calculate_step_rotation(rot)) 
    robot.MoveForward(dist, 1)
    env.step(utils.calculate_step_translation(dist))


    if(waypoints_data['bathing1']['rotation'] != "None"):
    # performing rotation action with required time step
        direction, rot  = utils.rotate(robot.data['rotation'][1], waypoints_data['bathing1']['rotation'])
        if direction == "Left" :
            robot.TurnLeft(rot, 1)
        else:
            robot.TurnRight (rot, 1)
        env.step(utils.calculate_step_rotation(rot))


    fine_tune_position = [robot.data['position'][0], 0.8 + 0.2, nose[2]]  # Position above the sponge at safe height
    print(f"Using IK to position above the sponge: {fine_tune_position}")
    robot.IKTargetDoMove(
        position=fine_tune_position,
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()

    print(f"Lowering gripper to bath")
    robot.IKTargetDoMove(
        position=[robot.data['position'][0], 1, nose[2]],  
        duration=1,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(100)
    # moving forward
    robot.MoveForward(1, 1)
    env.step(utils.calculate_step_translation(1))
    env.Pend()





if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    parser.add_argument('-d', '--dev', action='store_true', help='Run in developer mode')
    args = parser.parse_args()
    iteration = 20
    score_table= {}
    _main(use_graphics=args.graphics, dev=args.dev)
    # for i in range(iteration):
    #     _main(use_graphics=args.graphics, dev=args.dev)
    #     with open('/home/pszam2/.config/unity3d/RCareWorld/DressingPlayer/spongeScore.json') as f:
    #         score = json.load(f)
    #     score_table["trial" + str(i)]= score
    # # Convert into JSON
    # # File name is mydata.json
    # with open("score.json", "w") as final:
	#     json.dump(score_table, final)
