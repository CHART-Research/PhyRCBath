from pyrcareworld.envs.bathing_env import BathingEnv
import pyrcareworld.attributes.camera_attr as attr
import numpy as np
import utils

class Navigation:

    def __init__(self, env: BathingEnv, robot):
        self.environment = env
        self.robot =  robot



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
        self.environment.step(utils.calculate_step_rotation(rot)) 
        self.robot.MoveForward(dist, 1)
        self.environment.step(utils.calculate_step_translation(dist))

        if(target_rotation!= None):
        # performing rotation action with required time step
            direction, rot  = utils.rotate(self.robot.data['rotation'][1], target_rotation)
            if direction == "Left" :
                self.robot.TurnLeft(rot, 1)
            else:
                self.robot.TurnRight (rot, 1)
            self.environment.step(utils.calculate_step_rotation(rot))


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