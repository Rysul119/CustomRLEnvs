import gym
from gym import spaces

import pybullet as p
import pybullet_data
from pybullet_utils import bullet_client

import numpy as np
import os

class simpleEnvBot(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self, action_dim, obs_dim):
        self._p = bullet_client.BulletClient(connection_mode=p.GUI)  # to connect to a renderable server
        # defining action and observation space
        high = np.ones([action_dim])
        self.action_space = spaces.Box(-high, high)
        high = np.inf * np.ones([obs_dim])
        self.observation_space = spaces.Box(-high, high)

        # self.jointDict = {} # may not need
        # self.linkDict = {} # may not need
        # self.orderedJoints = [] # may not need

        self.dt = 1 / 240  # may have to modify?
        self.power = 50  # hard coding the power value


    def reset(self):
        self._p.resetSimulation()
        self._p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        self._p.setGravity(0, 0, -10)
        p.setTimeStep(self.dt)
        urdfRootPath = ""
        _, self.botId = p.loadMJCF(os.path.join(urdfRootPath, "simpleLinks.xml"))

        self._p.resetJointState(self.botId, 0,np.random.uniform(low= -1, high = 1) ,0)
        baseXYZ = self._p.getBasePositionAndOrientation(self.botId)[0] # xyz coordinates of the base
        self.counter = 0
        self._p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        return baseXYZ # using the base position as the robot position

    def step(self,action):
        torque = self.power * 20 * float(np.clip(action[0], -1, 1))
        self._p.setJointMotorControl2(bodyIndex = self.botId, jointIndex = 0, controlMode = p.TORQUE_CONTROL, force = torque)
        self._p.stepSimulation()
        state = self._p.getBasePositionAndOrientation(self.botId)[0]
        reward = -1
        done = False
        self.counter += 1
        if(state[0] == 1000 or self.counter == 1000):
            done = True

        return state, reward, done, {}

    def render(self):
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0.7, 0, 0.05],
                                                          distance=.7,
                                                          yaw=90,
                                                          pitch=-70,
                                                          roll=0,
                                                          upAxisIndex=2)
        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                   aspect=float(960) / 720,
                                                   nearVal=0.1,
                                                   farVal=100.0)
        (_, _, px, _, _) = p.getCameraImage(width=960,
                                            height=720,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (720, 960, 4))

        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        self._p.disconnect()