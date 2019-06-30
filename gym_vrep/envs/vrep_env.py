# -*- coding: utf-8 -*-
"""
Created on Fri May 17 21:02:20 2019

@author: Srn
"""

# -*- coding: utf-8 -*-
"""
Created on Wed May 15 15:05:59 2019

@author: Srn

@some parameters: slabstone size: 0.5x0.5x0.05  reference frame original point: 0.25x0.25x0.025
                  initial slabstone center:(0.65, 0.75, 1.4)
                  left grasping point: (0.45, 0.9,1.4)  right grasping point:(0.45, 0.6,1.4)
                  approach vector can calcutalate: z = 0.65-0.45-ik_shift-0.05/2  
                  when ik_shift=0.11, z = 0.065, but z can use a lewer value. so 0.063<=z<0.065,
                  approach vector is [0, 0, z]
"""

#import b0RemoteApi
from gym_vrep.envs import b0RemoteApi
import time
import math
import numpy as np
import os

# import gym api
import gym
from gym import error, spaces, utils
from gym.utils import seeding
from gym import logger

### 绘图
#import matplotlib
#import matplotlib.pyplot as plt
#
## %matplotlib inline
#
## set up matplotlib
#is_ipython = 'inline' in matplotlib.get_backend()
#if is_ipython:
#    from IPython import display
#
#plt.ion()
#
#
#def plot_durations(y):
#    plt.figure(1, figsize=(10, 10))
#    plt.clf()
#    ax1 = plt.subplot(2, 2, 1)
#    ax2 = plt.subplot(2, 2, 2)
#    ax3 = plt.subplot(2, 2, 3)
#    ax4 = plt.subplot(2, 2, 4)
#    plt.sca(ax1)
#    plt.title('road load rate ')
#    plt.xlabel('time slice')
#    plt.ylabel('load rate')
#    plt.plot(y[:, 0], y[:, 1])
#
#    plt.sca(ax2)
#    plt.title('cars to go ')
#    plt.xlabel('time slice')
#    plt.ylabel('cars num')
#    plt.plot(y[:, 0], y[:, 2])
#
#    plt.sca(ax3)
#    plt.title('cars on road ')
#    plt.xlabel('time slice')
#    plt.ylabel('cars on road')
#    plt.plot(y[:, 0], y[:, 3])
#
#    plt.sca(ax4)
#    plt.title('cars arrived ')
#    plt.xlabel('time slice')
#    plt.ylabel('cars arrived num')
#    plt.plot(y[:, 0], y[:, 4])
#
#    plt.pause(0.001)  # pause a bit so that plots are updated
#    if is_ipython:
#        display.clear_output(wait=True)
#        display.display(plt.gcf())

class Vrep_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        """
        initialize the b0_remote api client and start the simulaiton
        """
        try:
            self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi',60)
        except:
            raise Exception('connot connect to the vrep server')
        # info
        logger.set_level(20)
        logger.info('gym_vrep located at: ' + str(os.path.realpath('./')))
        
        # a simple communication test
        self.client.simxAddStatusbarMessage('Hello',self.client.simxDefaultPublisher()) 
        
        # start simulation
        self.client.simxStartSimulation(self.client.simxServiceCall())
        
        self.l_name = 'LBR4p'
        self.r_name = 'LBR4p#0'

        self.delta = 0.01 #  distance that move along x in every step, unit is m   # max 0.25
        self.theta = 0.5 / 180 * math.pi # angle that rotate along y,z in every step, unit is rad  # max (15 / 180 * math.pi) 
        
        # calculate the motion along x,y,z for slabstone
        self.move_distance_x = 0
        self.rotate_angle_y = 0
        self.rotate_angle_z = 0

        # threshold value for motion along x,y,z
        self.x_threshold_dis = 0.085 #0.25
        self.y_threshold_angle = 8 / 180 * math.pi  # 15
        self.z_threshold_angle = 8 / 180 * math.pi # 15

        # RL state, action, reward
        # TODO: to set this two threshold
        force_threshold = 300
        torque_threshold = 200
        high = np.array([force_threshold for _ in range(3)] + [torque_threshold for _ in range(3)] + 
                        [force_threshold for _ in range(3)] + [torque_threshold for _ in range(3)] )
        self.action_space = spaces.Discrete(6)  # 0,1: move laong x:+,-  2,3: rotate along y:+,-  4,5: rotate along z:+, -
        # two F/T sensor's data
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.seed()

        self.state = None    # state is 12x1 matrix
        self.step_beyond_done = None
        self.counts = 0
        self.counts_max = 30 # max adjust counts

        # some initial operation
        self.initial_simulation()
        self.initial_motion()

        # get initial state
        self.state = self.state_process()

        # count reset times
        self.reset_counter = 0


    
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def step(self, action):
        
        """
        7 actions
        0: not move 1,2: move laong x:+,-  3,4: rotate along y:+,-  4,6: rotate along z:+, -
        """
        assert self.action_space.contains(action), "%r (%s) invalid"%(action, type(action))
        state = self.state

        # # not move
        # if action == 0:
        #     self.state = state
        
        if action == 0:
            self.move_slabstone_along_x(self.delta)
        
        if action == 1:
            self.move_slabstone_along_x(-self.delta)
        
        if action == 2:
            self.rotate_slabstone_along_y(self.theta)
        
        if action == 3:
            self.rotate_slabstone_along_y(-self.theta)
        
        if action == 4:
            self.rotate_slabstone_along_z(self.theta)
        
        if action == 5:
            self.rotate_slabstone_along_z(-self.theta)
        
        # get new state
        self.state = self.state_process() 
        self.counts += 1

        done, reward = self.reward_func2(action)

        # print("reward:%f, done:",)
        # print('reward:%f, done:%s'%(reward, str(done)))
        logger.set_level(20)
        logger.info('action:%d, reward:%f, done:%s'%(action, reward, str(done)))


        return self.state, reward, done, {}


    
    def reset(self):
        # reset the calculation the motion along x,y,z for slabstone
        self.move_distance_x = 0
        self.rotate_angle_y = 0
        self.rotate_angle_z = 0

        self.counts = 0

        # self.initial_slab_pose = 
        self.reset_slabstone_to_initial()

        # get initial state
        self.state = self.state_process()

        return self.state

    
    def render(self, mode):
        return None

    def close(self):
        self.stop_simulation()
        return 1
    
    def reward_func(self, action):
        """
        reard function, calculate reward for current state and action
        """
        # left arm's force state 
        F1 = self.state[0:3]
        T1 = self.state[3:6]

        # right arm's force state 
        F2 = self.state[6:9]
        T2 = self.state[9:12]
        
        # some little value using for comparation
        d1 = 0.2  # Fz
        d2 = 0.2  # Tx
        d3 = 0.2  # Ty

        # set target force value along z-axis by experience, unit = N
        Ft = 30
        
        # not explore succeed
        if (self.counts > self.counts_max) or (self.move_distance_x > self.x_threshold_dis) or \
        (abs(self.rotate_angle_y) > self.y_threshold_angle) or (abs(self.rotate_angle_z) > self.z_threshold_angle):

           done = True
           reward = -1
        # installing succeed condition
        # use relative coordinates in end effector, that Z-axis ointing to the wall, X-axis is horizontal direction, 
        # and Y-axis is vertical direction
        # succeed condition: Fz1=Fz2=Ft, Tx1=Tx2=0, Ty1=-Ty2=0
        elif (abs(F1[2] - F2[2]) < d1) and (abs(F1[2] - Ft) < d1) and (abs(F2[2] - Ft) < d1) and \
            (abs(T1[0] - T2[0]) < d2) and (abs(T1[0]) < d2) and (abs(T2[0]) < d2) and \
            (abs(T1[1] + T2[1]) < d3) and (abs(T1[1]) < d3) and (abs(T2[1]) < d3): 

            done = True
            reward = 1
        
        elif (action == 1) and (self.move_distance_x < self.x_threshold_dis * 0.3):  # a guide reward to make slab move along x-axis in early time
            done = False
            reward = 0.05
        
        elif (action == 2) and (self.move_distance_x > self.x_threshold_dis * 0.7): # a punishment reward, restric to the motion along the  negative direction of x-axis
            done = False
            reward = -0.5
        
        else:  # continue to exploring, and give a little punishment reward
            done = False
            reward = -0.1

        return done, reward

    def reward_func1(self, action):
        """
        reard function, calculate reward for current state and action
        """
        # left arm's force state 
        F1 = self.state[0:3]
        T1 = self.state[3:6]

        # right arm's force state 
        F2 = self.state[6:9]
        T2 = self.state[9:12]
        
        # some little value using for comparation
        #d1 = 0.2  # Fz
        d1 = 5
        d2 = 0.2  # Tx
        d3 = 0.2  # Ty

        # set target force value along z-axis by experience, unit = N
        Ft = 30
        
        # not explore succeed
        if (self.counts > self.counts_max) or (self.move_distance_x > self.x_threshold_dis) or \
        (abs(self.rotate_angle_y) > self.y_threshold_angle) or (abs(self.rotate_angle_z) > self.z_threshold_angle):

           done = True
           reward = -1
        # installing succeed condition
        # use relative coordinates in end effector, that Z-axis ointing to the wall, X-axis is horizontal direction, 
        # and Y-axis is vertical direction
        # succeed condition: Fz1=Fz2=Ft, Tx1=Tx2=0, Ty1=-Ty2=0
        elif (abs(F1[2] - F2[2]) < d1) and (abs(F1[2] - Ft) < d1) and (abs(F2[2] - Ft) < d1): 
            done = True
            reward = 1
        
        elif (action == 1) and (self.move_distance_x < self.x_threshold_dis * 0.3):  # a guide reward to make slab move along x-axis in early time
            done = False
            reward = 0.05
        
        elif (action == 2) and (self.move_distance_x > self.x_threshold_dis * 0.7): # a punishment reward, restric to the motion along the  negative direction of x-axis
            done = False
            reward = -0.5
        
        else:  # continue to exploring, and give a little punishment reward
            done = False
            reward = -0.1

        return done, reward

    def reward_func2(self, action):
        """
        reard function, calculate reward for current state and action
        """
        
        # left arm's force state 
        F1 = self.state[0:3]
        T1 = self.state[3:6]

        # right arm's force state 
        F2 = self.state[6:9]
        T2 = self.state[9:12]

        print("F1[2]:%f, F2[2]:%f"%(F1[2],F2[2]))
        
        # some little value using for comparation
        #d1 = 0.2  # Fz
        d1 = 5
        d2 = 0.2  # Tx
        d3 = 0.2  # Ty

        # set target force value along z-axis by experience, unit = N
        Ft = 30
        
        # not explore succeed
        if (self.counts >= self.counts_max) or (abs(self.move_distance_x) > self.x_threshold_dis) or (abs(F1[2]+F2[2]) > 4*Ft) or \
        (abs(self.rotate_angle_y) > self.y_threshold_angle) or (abs(self.rotate_angle_z) > self.z_threshold_angle):
            done = True
            reward = -1
        # installing succeed condition
        # use relative coordinates in end effector, that Z-axis ointing to the wall, X-axis is horizontal direction, 
        # and Y-axis is vertical direction
        # succeed condition: Fz1=Fz2=Ft, Tx1=Tx2=0, Ty1=-Ty2=0
        elif (abs(F1[2] - F2[2]) < 10) and (abs(abs(F1[2]+F2[2]) - 2*Ft) < 10): 
            done = True
            reward = 1
        
        elif (action == 0) and (self.move_distance_x < self.x_threshold_dis * 0.2):  # a guide reward to make slab move along x-axis in early time
            done = False
            reward = 0.05
        
        elif (action == 1) and (self.move_distance_x > self.x_threshold_dis * 0.6): # a punishment reward, restric to the motion along the  negative direction of x-axis
            done = False
            reward = -0.3
        
        else:  # continue to exploring, and give a little punishment reward
            done = False
            reward = -0.1
        
        reward = reward + self.reward_attach(F1[2], F2[2], Ft)

        return done, reward
    
    def reward_attach(self, F1, F2, Ft):
        """
        attach reward according from distance between current state to target state
        """
        if F1 < 0 and F2 < 0:
            reward = (Ft - np.linalg.norm([abs(F1)-Ft, abs(F2)-Ft])) / Ft * 0.1
            if reward < -2 * 0.1:
                reward = -0.1
            return reward
        
        return 0
            
    def initial_simulation(self):
        """
        initialize the simulation, and get some objects handles
        """
        # get slabstone and slabstone's reference frame handle
        self.slab_h = self.client.simxGetObjectHandle("slabstone", self.client.simxServiceCall())[1]  # [True, 189]
        self.slab_rf_h = self.client.simxGetObjectHandle("slab_reference_frame", self.client.simxServiceCall())[1]
        # left reference grasping point handle
        self.slab_rf_l_h = self.client.simxGetObjectHandle("slab_reference_frame_L", self.client.simxServiceCall())[1]
        self.slab_rf_fl_h = self.client.simxGetObjectHandle("slab_reference_frame_fL", self.client.simxServiceCall())[1]
        # right reference grasping point handle
        self.slab_rf_r_h = self.client.simxGetObjectHandle("slab_reference_frame_R", self.client.simxServiceCall())[1]
        self.slab_rf_fr_h = self.client.simxGetObjectHandle("slab_reference_frame_fR", self.client.simxServiceCall())[1]
        
        
        # get two manipulators handles
        arm_name = 'LBR4p#'
        self.arm_l_h = self.client.simxGetObjectHandle("".join([arm_name, '']),
                                                       self.client.simxServiceCall())[1]  # [True, 55]
        self.arm_r_h = self.client.simxGetObjectHandle("".join([arm_name, '0']),
                                                       self.client.simxServiceCall())[1]  # [True, 78]
        
        # get target handle
        self.testTarget1_h = self.client.simxGetObjectHandle("testTarget1",
                                                             self.client.simxServiceCall())[1]  # [True, 135]
        self.testTarget2_h = self.client.simxGetObjectHandle("testTarget2",
                                                             self.client.simxServiceCall())[1]  # [True, 139]
        
        # get nine spring_dampers handles
        sd_h = []
        sd_name = 'SpringDamper_springAndDamper'
        temp = self.client.simxGetObjectHandle(sd_name, self.client.simxServiceCall())[1]
        sd_h.append(temp)
        
        for i in range(8):
            temp = self.client.simxGetObjectHandle("".join([sd_name, str(i)]),
                                                   self.client.simxServiceCall())[1]
            sd_h.append(temp)
        
        self.sd_h = sd_h
        
        # get left and right manipulator's force/torque sensor handles
        self.left_ft_sensor_h = self.client.simxGetObjectHandle("LBR4p_connection",
                                                             self.client.simxServiceCall())[1]
        self.right_ft_sensor_h = self.client.simxGetObjectHandle("LBR4p_connection#0",
                                                             self.client.simxServiceCall())[1]
        
        # get Cuboid3 handle, Cuboid3 is used to support the slabstone before suction gets it
        self.Cuboid3_h = self.client.simxGetObjectHandle("Cuboid3",
                                                             self.client.simxServiceCall())[1]
        
        # get length between grasping point and center of slabstone
        _, center_position = self.client.simxGetObjectPosition(self.slab_rf_h, -1,
                                                                self.client.simxServiceCall())
        _, center_orientation = self.client.simxGetObjectOrientation(self.slab_rf_h, -1,
                                                                self.client.simxServiceCall())
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        # L = y1-y2
        self.L = abs(center_position[1] - left_position[1])
        # get initial pose of slab
        self.initial_slab_pose = center_position + center_orientation
        
        # record initial position and oritation of left and right grasping points
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        _, left_orientation = self.client.simxGetObjectOrientation(self.slab_rf_l_h, -1,
                                                                    self.client.simxServiceCall())
        self.left_init_positon = left_position
        self.left_init_orientation = left_orientation
        
        _, right_position = self.client.simxGetObjectPosition(self.slab_rf_r_h, -1,
                                                                self.client.simxServiceCall())
        _, right_orientation = self.client.simxGetObjectOrientation(self.slab_rf_r_h, -1,
                                                                    self.client.simxServiceCall())
        self.right_init_positon = right_position
        self.right_init_orientation = right_orientation
        

    def initial_motion(self):
        """
        initial motion: move two arms to grasping point at same time
        """
        
        # some parameters use for path planning
        approachVector=[0,0,0.058] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
        searchCount=2 #2 # how many times OMPL will run for a given task
        minConfigsForPathPlanningPath= 50 #400 # interpolation states for the OMPL path
        minConfigsForIkPath=50 #100 # interpolation states for the linear approach path
        collisionChecking=1 # whether collision checking is on or off
        
        # get left arm's path
        current_config_l = self.__get_current_config(self.arm_l_h)
        target_m_l = self.__get_target_matrix(self.testTarget1_h)
        target_m_l = self.__get_matrix_shifted_along_Z(target_m_l, 0.11)#0.05)
        
        self.getPath_goalIsPose(self.arm_l_h, current_config_l, target_m_l,
                                approachVector,
                                maxConfigsForDesiredPose,
                                maxTrialsForConfigSearch,
                                searchCount,
                                minConfigsForPathPlanningPath,
                                minConfigsForIkPath,
                                collisionChecking)

        
        # get right arm's path
        current_config_r = self.__get_current_config(self.arm_r_h)
        target_m_r = self.__get_target_matrix(self.testTarget2_h)
        target_m_r = self.__get_matrix_shifted_along_Z(target_m_r, 0.11)#0.05)
        
        self.getPath_goalIsPose(self.arm_r_h, current_config_r, target_m_r,
                                approachVector,
                                maxConfigsForDesiredPose,
                                maxTrialsForConfigSearch,
                                searchCount,
                                minConfigsForPathPlanningPath,
                                minConfigsForIkPath,
                                collisionChecking)
        
        # move at same time
        self.client.simxSetIntSignal('start_move', 1, self.client.simxServiceCall())
        
        # wait until finish initial motion
        isFollowPathFinished = 0
        isFollowPathFinished1 = 0
        while isFollowPathFinished == 0 or isFollowPathFinished1 == 0:
            isFollowPathFinished = self.client.simxGetIntSignal('isFollowPathFinished#',
                                                                self.client.simxServiceCall())[1]
            isFollowPathFinished1 = self.client.simxGetIntSignal('isFollowPathFinished#0',
                                                                 self.client.simxServiceCall())[1]
            
            # delay a while
            time.sleep(0.05)  
        
        # move support cuboid after finish initial motion
        self.client.simxSetObjectPosition(self.Cuboid3_h, -1, [0.6,0.6,0.1],
                                          self.client.simxServiceCall())
        # set signal to 0
        self.client.simxSetIntSignal('isFollowPathFinished#', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('isFollowPathFinished#0', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('start_move', 0, self.client.simxServiceCall())
        
        print("initial motion finished")
                                     
        ## finish initial motion
        
    def getPath_goalIsPose(self, arm_h, current_config, target_m,
                           approachVector=[0,0,0], # approachVector
                           maxConfigsForDesiredPose=10, # maxConfigsForDesiredPose
                           maxTrialsForConfigSearch=200, # maxTrialsForConfigSearch
                           searchCount=2, # searchCount
                           minConfigsForPathPlanningPath=200, # minConfigsForPathPlanningPath
                           minConfigsForIkPath=50, # minConfigsForIkPath
                           collisionChecking=1): # collisionChecking
        """
        :param arm_h: handle of arm to execute motion
        :param current_config: current config of arm
        :param target_m: target point's pose matrix
        """
        
        # some parameters
#        approachVector=[0,0,0.058] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
#        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
#        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
#        searchCount=2 # how many times OMPL will run for a given task
#        minConfigsForPathPlanningPath=400 # interpolation states for the OMPL path
#        minConfigsForIkPath=50 #100 # interpolation states for the linear approach path
#        collisionChecking=1 # whether collision checking is on or off
        
        # Do the path planning here (between a start state and a goal pose, including a linear approach phase):
        inInts=[arm_h, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath,
                maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount]
        
        inFloats=current_config + target_m + approachVector

        self.client.simxCallScriptFunction('findPath_goalIsPose_remote_api@remoteApiCommandServer',
                           'sim.scripttype_childscript',
                           [inInts, inFloats],
                        #    client.simxDefaultPublisher())
                           self.client.simxServiceCall())
        
        # TODO: to modify 'isRunning' signal to make it useful
        self.client.simxSetIntSignal('isRunning', 1, self.client.simxServiceCall())
        
    
    def getIKPath(self, arm_h, fake_reference_h, cur_config, tar_position, tar_orientation,
                  approachVector=[0,0,0], # approachVector
                  maxConfigsForDesiredPose=10, # maxConfigsForDesiredPose
                  maxTrialsForConfigSearch=200, # maxTrialsForConfigSearch
                  searchCount=2, # searchCount
                  minConfigsForPathPlanningPath=200, # minConfigsForPathPlanningPath
                  minConfigsForIkPath=50, # minConfigsForIkPath
                  collisionChecking=1): # collisionChecking)
        
        # get target matrix by slab_reference_frame_fL/fR
        self.client.simxSetObjectPosition(fake_reference_h, -1, tar_position,
                                          self.client.simxServiceCall())
        self.client.simxSetObjectOrientation(fake_reference_h, -1, tar_orientation,
                                             self.client.simxServiceCall())
        target_m = self.__get_target_matrix(fake_reference_h)
        
#        self.getPath_goalIsPose(self.arm_l_h, current_config_l, target_m_l,
        res = self.getIKPath_goalIsPose(arm_h, cur_config, target_m,
                                approachVector,
                                maxConfigsForDesiredPose,
                                maxTrialsForConfigSearch,
                                searchCount,
                                minConfigsForPathPlanningPath,
                                minConfigsForIkPath,
                                collisionChecking)
        return res
        
                           
    
    def getIKPath_goalIsPose(self, arm_h, current_config, target_m,
                           approachVector=[0,0,0], # approachVector
                           maxConfigsForDesiredPose=10, # maxConfigsForDesiredPose
                           maxTrialsForConfigSearch=200, # maxTrialsForConfigSearch
                           searchCount=2, # searchCount
                           minConfigsForPathPlanningPath=200, # minConfigsForPathPlanningPath
                           minConfigsForIkPath=50, # minConfigsForIkPath
                           collisionChecking=1): # collisionChecking
        """
        :param arm_h: handle of arm to execute motion
        :param current_config: current config of arm
        :param target_m: target point's pose matrix
        """
        
        # Do the path planning here (between a start state and a goal pose, including a linear approach phase):
        inInts=[arm_h, collisionChecking, minConfigsForIkPath, minConfigsForPathPlanningPath,
                maxConfigsForDesiredPose, maxTrialsForConfigSearch, searchCount]
        
        inFloats=current_config + target_m + approachVector

        _, path_length, _, _ = self.client.simxCallScriptFunction('findIKPath_goalIsPose_remote_api@remoteApiCommandServer',
                           'sim.scripttype_childscript',
                           [inInts, inFloats],
                        #    client.simxDefaultPublisher())
                           self.client.simxServiceCall())
        
        # TODO: to modify 'isRunning' signal to make it useful
        self.client.simxSetIntSignal('isRunning', 1, self.client.simxServiceCall())

        return path_length>0
    
    
    def set_slabstone_pose(self, postion, orientation):
        """
        set slabstone pose, and map this pose to two manipulator's motion
        only move slabstone along Z-axis, and rotate it along X&Y-axis
        """
        pass
    
    # action of slabstone
    def move_slabstone_along_x(self, delta):
        """
        move slab alone x-axis in absolute coordinate, and it will move along z-axis in slab coordinate
        +delta and -delta are two actions
        :param delta: move distance, unit is meter
        """
        # some parameters use for path planning
        approachVector=[0,0,0] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
        searchCount=160 #2 # how many times OMPL will run for a given task
        minConfigsForPathPlanningPath=20 # interpolation states for the OMPL path
        minConfigsForIkPath=40 #60 #100 # interpolation states for the linear approach path
        collisionChecking=0 # whether collision checking is on or off
        
        ##########
        # path planning for left arm
        ##########
        # get current config of left arm
        current_config_l = self.__get_current_config(self.arm_l_h)
        
        # get left grasping point reference position and orientation in absolute coordinate
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        _, left_orientation = self.client.simxGetObjectOrientation(self.slab_rf_l_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new left position
        # only need change x
        left_position[0] += delta
        
        res = self.getIKPath(self.arm_l_h, self.slab_rf_fl_h, current_config_l, left_position, left_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        
        #print(res)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0
        ##########
        # path planning for right arm
        ##########
        # get current config of right arm
        current_config_r = self.__get_current_config(self.arm_r_h)
        
        # get right grasping point reference position and orientation in absolute coordinate
        _, right_position = self.client.simxGetObjectPosition(self.slab_rf_r_h, -1,
                                                                self.client.simxServiceCall())
        _, right_orientation = self.client.simxGetObjectOrientation(self.slab_rf_r_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new right position
        # only need change x
        right_position[0] += delta
        
        res = self.getIKPath(self.arm_r_h, self.slab_rf_fr_h, current_config_r, right_position, right_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        
        # move at same time
#        print('start running')
        # input('start moving along x-axis')
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0

        # start moving 
        self.client.simxSetIntSignal('start_move', 1, self.client.simxServiceCall())
        
        # wait until finish initial motion
        isFollowPathFinished = 0
        isFollowPathFinished1 = 0
        while isFollowPathFinished == 0 or isFollowPathFinished1 == 0:
            isFollowPathFinished = self.client.simxGetIntSignal('isFollowPathFinished#',
                                                                self.client.simxServiceCall())[1]
            isFollowPathFinished1 = self.client.simxGetIntSignal('isFollowPathFinished#0',
                                                                 self.client.simxServiceCall())[1]
            
            # delay a while
            time.sleep(0.05)  
        

        # set signal to 0
        self.client.simxSetIntSignal('isFollowPathFinished#', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('isFollowPathFinished#0', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('start_move', 0, self.client.simxServiceCall())
        
        # update calculation
        self.move_distance_x += delta
        print("move %f along x-axis finished" %delta)
                                     
        ## finish motion along x-axis
    
    # TODO: this function has some bugs to fix
    def rotate_slabstone_along_y(self, theta):
        """
        rotate slabstone along y-axis in absolute coordinate, and it will rotate along x-axis in slab coordinate
        +theta and -theta are two actions
        :param theta: rotate angle, unit is rad
        """
        # some parameters use for path planning
        approachVector=[0,0,0] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
        searchCount=160 #2 # how many times OMPL will run for a given task
        minConfigsForPathPlanningPath=20 # interpolation states for the OMPL path
        minConfigsForIkPath=40 #60 #100 # interpolation states for the linear approach path
        collisionChecking=0 # whether collision checking is on or off
        
        
        ##########
        # path planning for left arm
        ##########
        # get current config of left arm
        current_config_l = self.__get_current_config(self.arm_l_h)
        
        # get left grasping point reference position and orientation in absolute coordinate
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        _, left_orientation = self.client.simxGetObjectOrientation(self.slab_rf_l_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new left position
        # only need change theta along y-axis
        left_orientation[1] += theta
        
        res = self.getIKPath(self.arm_l_h, self.slab_rf_fl_h, current_config_l, left_position, left_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0
        
        ##########
        # path planning for right arm
        ##########
        # get current config of right arm
        current_config_r = self.__get_current_config(self.arm_r_h)
        
        # get right grasping point reference position and orientation in absolute coordinate
        _, right_position = self.client.simxGetObjectPosition(self.slab_rf_r_h, -1,
                                                                self.client.simxServiceCall())
        _, right_orientation = self.client.simxGetObjectOrientation(self.slab_rf_r_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new right position and orientation  
        # only need change x
        right_orientation[1] += theta
        
        res = self.getIKPath(self.arm_r_h, self.slab_rf_fr_h, current_config_r, right_position, right_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0
        
        # move at same time
#        print('start running')
        # input('start rotating along y-axis')
        self.client.simxSetIntSignal('start_move', 1, self.client.simxServiceCall())
        
        # wait until finish initial motion
        isFollowPathFinished = 0
        isFollowPathFinished1 = 0
        while isFollowPathFinished == 0 or isFollowPathFinished1 == 0:
            isFollowPathFinished = self.client.simxGetIntSignal('isFollowPathFinished#',
                                                                self.client.simxServiceCall())[1]
            isFollowPathFinished1 = self.client.simxGetIntSignal('isFollowPathFinished#0',
                                                                 self.client.simxServiceCall())[1]
            
            # delay a while
            time.sleep(0.05)  
        

        # set signal to 0
        self.client.simxSetIntSignal('isFollowPathFinished#', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('isFollowPathFinished#0', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('start_move', 0, self.client.simxServiceCall())
        
        # update calculation
        self.rotate_angle_y += theta
        print("rotate %f along y-axis finished" % theta)
                                     
        ## finish rotation along y-axis
    
    def rotate_slabstone_along_z(self, theta):
        """
        rotate slabstone along z-axis in absolute coordinate, and it will rotate along y-axis in slab coordinate
        +theta and -theta are two actions
        :param theta: rotate angle, unit is rad
        """
        # some parameters use for path planning
        approachVector=[0,0,0] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
        searchCount=160 #2 # how many times OMPL will run for a given task
        minConfigsForPathPlanningPath=20 # interpolation states for the OMPL path
        minConfigsForIkPath=40 #60 #100 # interpolation states for the linear approach path
        collisionChecking=0 # whether collision checking is on or off
        
        
        ##########
        # path planning for left arm
        ##########
        # get current config of left arm
        current_config_l = self.__get_current_config(self.arm_l_h)
        
        # get left grasping point reference position and orientation in absolute coordinate
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        _, left_orientation = self.client.simxGetObjectOrientation(self.slab_rf_l_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new left position and orientation 
        # need change theta along z-axis, and change x,y
        left_orientation[2] += theta
        left_position[0] -= self.L * math.sin(theta)
        left_position[1] -= self.L * (1 - math.cos(theta))
        
        res = self.getIKPath(self.arm_l_h, self.slab_rf_fl_h, current_config_l, left_position, left_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0
        
        ##########
        # path planning for right arm
        ##########
        # get current config of right arm
        current_config_r = self.__get_current_config(self.arm_r_h)
        
        # get right grasping point reference position and orientation in absolute coordinate
        _, right_position = self.client.simxGetObjectPosition(self.slab_rf_r_h, -1,
                                                                self.client.simxServiceCall())
        _, right_orientation = self.client.simxGetObjectOrientation(self.slab_rf_r_h, -1,
                                                                    self.client.simxServiceCall())
        
        # new right position and orientation 
        # need change theta along z-axis, and change x,y
        right_orientation[2] += theta
        right_position[0] += self.L * math.sin(theta)
        right_position[1] += self.L * (1 - math.cos(theta))
        
        res = self.getIKPath(self.arm_r_h, self.slab_rf_fr_h, current_config_r, right_position, right_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            return 0

        # move at same time
#        print('start running')
        # input('start rotating along z-axis')
        self.client.simxSetIntSignal('start_move', 1, self.client.simxServiceCall())
        
        # wait until finish initial motion
        isFollowPathFinished = 0
        isFollowPathFinished1 = 0
        while isFollowPathFinished == 0 or isFollowPathFinished1 == 0:
            isFollowPathFinished = self.client.simxGetIntSignal('isFollowPathFinished#',
                                                                self.client.simxServiceCall())[1]
            isFollowPathFinished1 = self.client.simxGetIntSignal('isFollowPathFinished#0',
                                                                 self.client.simxServiceCall())[1]
            
            # delay a while
            time.sleep(0.05)  
        

        # set signal to 0
        self.client.simxSetIntSignal('isFollowPathFinished#', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('isFollowPathFinished#0', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('start_move', 0, self.client.simxServiceCall())
        
        # update calculation
        self.rotate_angle_z += theta
        print("rotate %f along z-axis finished" % theta)
                                     
        ## finish rotation along z-axis
    
    def reset_slabstone_to_initial(self, if_random=False):
        """
        move slabstone to initial pose from current pose
        """
        # if reset several times, use self.reset_simulation() to reset simulation to decrease shift
        self.reset_counter += 1
        if self.reset_counter >= 15:
            self.reset_counter = 0
            try:
                self.reset_simulation()
                print("reset slabstone pose finished")
                return 0
            except:
                print("reset slabstone pose failed")
                raise Exception("reset slabstone pose failed")
                return 0

        # some parameters use for path planning
        approachVector=[0,0,0] #[0,0,0.063] # often a linear approach is required. This should also be part of the calculations when selecting an appropriate state for a given pose
        maxConfigsForDesiredPose=20 # will try to find 10 different states corresponding to the goal pose and order them according to distance from initial state
        maxTrialsForConfigSearch=300 # a parameter needed for finding appropriate goal states
        searchCount=160 #2 # how many times OMPL will run for a given task
        minConfigsForPathPlanningPath=20 # interpolation states for the OMPL path
        minConfigsForIkPath=60 #100 # interpolation states for the linear approach path
        collisionChecking=0 # whether collision checking is on or off
        
        ##########
        # path planning for left arm
        ##########
        # get current config of left arm
        current_config_l = self.__get_current_config(self.arm_l_h)
        
        # get left grasping point reference position and orientation in absolute coordinate
        _, left_position = self.client.simxGetObjectPosition(self.slab_rf_l_h, -1,
                                                                self.client.simxServiceCall())
        _, left_orientation = self.client.simxGetObjectOrientation(self.slab_rf_l_h, -1,
                                                                    self.client.simxServiceCall())
        
        res = self.getIKPath(self.arm_l_h, self.slab_rf_fl_h, current_config_l, self.left_init_positon, self.left_init_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            try:
                self.reset_simulation()
                print("reset slabstone pose finished")
                return 0
            except:
                print("reset slabstone pose failed")
                raise Exception("reset slabstone pose failed")
                return 0
        
        ##########
        # path planning for right arm
        ##########
        # get current config of right arm
        current_config_r = self.__get_current_config(self.arm_r_h)
        
        # get right grasping point reference position and orientation in absolute coordinate
        _, right_position = self.client.simxGetObjectPosition(self.slab_rf_r_h, -1,
                                                                self.client.simxServiceCall())
        _, right_orientation = self.client.simxGetObjectOrientation(self.slab_rf_r_h, -1,
                                                                    self.client.simxServiceCall())
        
        res = self.getIKPath(self.arm_r_h, self.slab_rf_fr_h, current_config_r, self.right_init_positon, self.right_init_orientation,
                       approachVector,
                       maxConfigsForDesiredPose,
                       maxTrialsForConfigSearch,
                       searchCount,
                       minConfigsForPathPlanningPath,
                       minConfigsForIkPath,
                       collisionChecking)
        # judge if get the valid path
        # if res is false, the path not found, and return before the move
        if not res:
            try:
                self.reset_simulation()
                print("reset slabstone pose finished")
                return 0
            except:
                print("reset slabstone pose failed")
                raise Exception("reset slabstone pose failed")
                return 0
        # move steps: move along x-axis first, the rotate along y-axis and z-axis
        
        # move at same time
#        print('start running')
        # input('reset slabstone pose')
        self.client.simxSetIntSignal('start_move', 1, self.client.simxServiceCall())
        
        # wait until finish initial motion
        isFollowPathFinished = 0
        isFollowPathFinished1 = 0
        while isFollowPathFinished == 0 or isFollowPathFinished1 == 0:
            isFollowPathFinished = self.client.simxGetIntSignal('isFollowPathFinished#',
                                                                self.client.simxServiceCall())[1]
            isFollowPathFinished1 = self.client.simxGetIntSignal('isFollowPathFinished#0',
                                                                 self.client.simxServiceCall())[1]
            
            # delay a while
            time.sleep(0.05)  
        

        # set signal to 0
        self.client.simxSetIntSignal('isFollowPathFinished#', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('isFollowPathFinished#0', 0, self.client.simxServiceCall())
        self.client.simxSetIntSignal('start_move', 0, self.client.simxServiceCall())
        
        print("reset slabstone pose finished")
                                     
        ## finish reset slabstone pose
        
    
    def set_spring_damper_state(self, length=[0,0,0,0,0,0,0,0,0]):
        """
        there are 9 spring_dampers to simulate wall in vrep scene,
        this function is to adjust different length of spring_dampers 
        to make different situation of contact face
        """
        if len(length) != 9:
            raise Exception('input invalid spring_damper length parameter')
        
        # if length =0, no need to set
        if sum(length) == 0:
            return 1
        
        # set spring_damper length
        for i, sd_h in enumerate(self.sd_h):
            self.client.simxSetJointTargetPosition(sd_h, length[i], self.client.simxServiceCall())
    
    
    def get_ft_state(self):
        """
        get force/torque data from left and right manipulators' F/T sensor
        """
        left_res = self.client.simxReadForceSensor(self.left_ft_sensor_h, self.client.simxServiceCall())
        right_res = self.client.simxReadForceSensor(self.right_ft_sensor_h, self.client.simxServiceCall())
#        return: [True, 1, [-0.0007007975364103913, -0.0010367694776505232, -0.490450382232666],
#        [8.96616475074552e-05, 1.5039358004287351e-05, 2.193332022670802e-07]] [True, 1,
#        [0.4769093990325928, -0.04762545973062515, -0.0013335853582248092], [0.00167963
#        21142464876, 0.01667582243680954, 5.557537224376574e-05]]
        # print(left_res, right_res)
        return left_res, right_res
    
    def state_process(self):
        """
        using mean filter for F/T data
        """
        temp = []
        left_res, right_res = self.get_ft_state()
        if left_res[0] == True and right_res[0] == True:
            temp.append(left_res[2]+left_res[3]+right_res[2]+right_res[3])
        
        # get enough data
        i = 0
        while len(temp) <= 5:
            time.sleep(0.02)  # sample time
            left_res, right_res = self.get_ft_state()
            if left_res[0] == True and right_res[0] == True:
                temp.append(left_res[2]+left_res[3]+right_res[2]+right_res[3])
            
            
            i += 1
            if i > 20:
                raise Exception("cannot get enough sensor's data")
        
    
        # mean filter, temp is 5x12 dimension list
        temp_n = np.array(temp, dtype=float)
        
        state_mean = temp_n.mean(axis=0)
        # print(state_mean)
        
        return state_mean
    
    
    def print_object_handle(self):
        """print some objects handles"""
        print("slab_h:", self.slab_h)  
        print("arm_l_h,arm_r_h:", self.arm_l_h,self.arm_r_h)
        print("testTartget1_h:", self.testTarget1_h)
        print("testTartget2_h:", self.testTarget2_h)
        print("spring_damper_h:", self.sd_h)
        print("left and right f/t sensor:", self.left_ft_sensor_h, self.right_ft_sensor_h)

    
    def stop_simulation(self):
        """
        must use this function to exit and stop vrep manually
        """
        self.client.simxStopSimulation(self.client.simxServiceCall())
        self.__exit_env()
        print("Exiting V-rep and B0-Remote API")
    
    def reset_simulation(self):
        """
        if reset_slabstone_to_initial function not work because of ik
        path not found, use this function to reset the simulation by restart
        the simulaiton
        """
        self.client.simxStopSimulation(self.client.simxServiceCall())

        time.sleep(0.5)

        # a simple communication test
        self.client.simxAddStatusbarMessage('Hello',self.client.simxDefaultPublisher()) 
        
        # start simulation
        self.client.simxStartSimulation(self.client.simxServiceCall())

        self.initial_motion()
        self.state = self.state_process()
        
    
    def __get_current_config(self, arm_h):
        """
        get current config of arm, unit is rad
        :param arm_h:  arm handle
        :return: config
        """
        _, init_state = self.client.simxCallScriptFunction('getRobotState@remoteApiCommandServer',
                                                      'sim.scripttype_childscript',
                                                      arm_h,
                                                      self.client.simxServiceCall())
        return init_state
        
    def __get_target_matrix(self, target_h):
        """
        get target point's pose matrix
        :param target_h: target point hanfle
        :return : 1x12 matrix
        """
        _, target_m = self.client.simxGetObjectMatrix(target_h, -1, self.client.simxServiceCall())
        return target_m
    
    def __get_matrix_shifted_along_Z(self, m, local_z_shift):
        """
        Returns a pose or matrix shifted by localZShift along the matrix's z-axis
        """
        shift_m = m
        
        # shift along z
        shift_m[3] += shift_m[2] * local_z_shift
        shift_m[7] += shift_m[6] * local_z_shift
        shift_m[11] += shift_m[10] * local_z_shift
        
        return shift_m
        
        
    def __exit_env(self):
        """
        destroy and exit b0_remote api, 
        same as "with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi',60) as client: "
        """
        self.client.__exit__()
        
        

if __name__ == "__main__":
    env = Vrep_Env()
    env.state_process()
    env.move_slabstone_along_x(0.05)
    env.close()
    
#     env.initial_simulation()
   
    
#     env.print_object_handle()
# #    input('pause')
#     env.set_spring_damper_state([0.02,0,0,0,0,0,0,0,0])
#     env.get_ft_state()
    
#     env.initial_motion()
    
#     env.move_slabstone_along_x(0.05)
#     env.rotate_slabstone_along_z(-10/180*math.pi)
    
# #    env.move_slabstone_along_x(0.07)
    
#     env.reset_slabstone_to_initial()
    
# #    env.rotate_slabstone_along_z(10/180*math.pi)
# #    
# #    env.move_slabstone_along_x(0.04)
# #    
# #    env.move_slabstone_along_x(-0.1)
# #    
# #    env.reset_slabstone_to_initial()
    
#     startTime=time.time()
#     while time.time()<startTime+40:
#         pass
    
    
#    env.stop_simulation()
