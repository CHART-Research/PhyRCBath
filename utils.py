import numpy as np
import math

def manikin_randomizer(randomize = False): 
    """
    Return a random seed to randomize the manikin pose
    :param randomize:  Bool
    :return: int with seed value if any
    """
    if(randomize == False):
        return None
    else:
        return np.random.randint(0, 100) #random pose of the manikin


def random_robot_pose(robot): 
    """
    Set a random robot position [+- 25 cm] and orientation [+- 50 deg]
    :param robot: robot object
    :return: robot object with updated position and rotation
    """
    #random starting position of the robot within a square of 0.5x0.5
    randomX = np.random.uniform(0, 1) * 0.5 - 0.25
    randomZ = np.random.uniform(0, 1) * 0.5 - 0.25
    randomR = np.random.randint(0, 100) - 50
    robot_pos = robot.data['position']
    robot_orientation = robot.data['rotation']
    new_position = [robot_pos[0]+randomX, robot_pos[1], robot_pos[2]+randomZ]
    new_rotation = [robot_orientation[0], robot_orientation[1]+randomR, robot_orientation[2]]
    robot.SetPosition(new_position, True)
    robot.SetRotation(new_rotation, True)
    return robot

def random_sponge_pose(sponge): 
    """
    Set a random robot position [+- 10 cm] and orientation [+- 45 deg]
    :param robot: robot object
    :return: robot object with updated position and rotation
    """
    randomX = np.random.uniform(0, 1)*0.02 - 0.01
    randomZ = np.random.uniform(0, 1)*0.02  - 0.01
    randomR = np.random.randint(0, 90) - 45
    sponge_pos = sponge.data['position']
    sponge_orientation = sponge.data['rotation']
    new_position = [sponge_pos[0]+randomX, sponge_pos[1], sponge_pos[2]+randomZ]
    new_rotation = [sponge_orientation[0], sponge_orientation[1]+randomR, sponge_orientation[2]]
    sponge.SetPosition(new_position, True)
    sponge.SetRotation(new_rotation, True)
    return sponge

def calculate_distance (pointA, pointB):
    """
    Calculate the distance between two points in the horizontal plane
    :param pointA, pointB: list [x,y,z]
    :return: float distance
    """
    return math.sqrt((pointA[0]-pointB[0])*(pointA[0]-pointB[0]) + (pointA[2]-pointB[2])*(pointA[2]-pointB[2]))



def move(current, target, current_rotation):
    """
    Calculate the parameter to move from current point to target point
    :param current, target: list [x,y,z]
    :return: the turning direction,  the rotation and the distance
    """
    distance = calculate_distance(target, current)
    deltaX = target[0] - current[0]
    deltaY = target[2] - current[2]
    theta = math.atan2(deltaY, deltaX) * 180/math.pi
    if(theta > 0):
        target_rotation = (theta + current_rotation -90) % 360
        if target_rotation > 180:
            turn = "Right"
            print("target_rotation:", target_rotation), 
            target_rotation = 360 - target_rotation
        else:
            turn = "Left"

    else:
        target_rotation = ( abs(theta)- current_rotation + 90) % 360
        if target_rotation > 180:
            turn = "Left"
            target_rotation = 360 - target_rotation
        else:
            turn = "Right"
    return turn, target_rotation, distance 


def rotate(current_rotation, target):
    """
    Calculate the parameter to move from current point to target point
    :param current, target: list [x,y,z]
    :return: the turning direction,  the rotation and the distance
    """
    theta = target - current_rotation
    if theta < 0:
        theta += 360
    print("theta: ", theta)
    if theta > 180:
        turn = "Left"
        theta = 360 - theta
    else:
        turn = "Right"
    return turn, theta


def calculate_step_rotation(rotation):
    """
    Calculate the  simulation time step required for rotation. Empirically 90 degree requires 300 (1 rad/s)
    :param rotation: rotation 
    :return: time step as an int
    """
    return int (rotation * 300/90 + 100)


def calculate_step_translation(distance):
    """
    Calculate the  simulation time step required for travelling a certain distance. (1 m/s) Empirically ....
    :param rotation: rotation 
    :return: time step as an int
    """
    return int(distance * 400) 
