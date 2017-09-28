#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from math import *
import numpy as np
import json

MAX_X = 200
MAX_Y = 40
LANE_WIDTH = 3

class WorldGenerator:
    def __init__(self, path2config):
        self.path2config = path2config
        
        self.loadConfig()
        self.carInitPositions()
        self.write2WorldFile()

    def loadConfig(self):
        print self.path2config
        with open(self.path2config, 'r') as f:
            self.d = json.load(f)

        self.world_name = self.d['world_name']
        self.path2world = self.d['path_world']
        self.bir_direction = self.d['bir_direction']
        self.num_lanes = self.d['num_lanes']
        self.bottom_space = self.d['bottom_space']
        self.num_cars_per_lane = self.d['num_cars_per_lane']
        self.car_position_offset = self.d['car_position_offset']
        self.car_position_interval = self.d['car_position_interval']
        # self.mean_speed_lane = self.d['mean_speed_lane']
        # self.var_speed_lane = self.d['var_speed_lane']

    def carInitPositions(self):
        self.cars = {}
        self.cars['name'] = []
        self.cars['id'] = []
        self.cars['init_poses'] = []
        self.cars['init_dir'] = []

        if self.bir_direction == 0:
            # all car driving to the left
            count = 1
            for i in range(self.num_lanes):
                num_cars_lane = self.num_cars_per_lane[i]
                for j in range(num_cars_lane):
                    self.cars['name'].append('gc2' + str(count))
                    self.cars['id'].append(count)
                    x = MAX_X - self.car_position_offset[i]\
                            - j * self.car_position_interval[i]
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    self.cars['init_poses'].append(pose)
                    self.cars['init_dir'].append(180)
                    count += 1

        if self.bir_direction == 1:
            # bottom 5 lanes driving to the left
            count = 1
            for i in range(self.num_lanes/2):
                num_cars_lane = self.num_cars_per_lane[i]
                for j in range(num_cars_lane):
                    self.cars['name'].append('gc2' + str(count))
                    self.cars['id'].append(count)
                    x = MAX_X - self.car_position_offset[i]\
                            - j * self.car_position_interval[i]
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    self.cars['init_poses'].append(pose)
                    self.cars['init_dir'].append(180)
                    count += 1

            for i in range(self.num_lanes/2, self.num_lanes):
                num_cars_lane = self.num_cars_per_lane[i]
                for j in range(num_cars_lane):
                    self.cars['name'].append('gc2' + str(count))
                    self.cars['id'].append(count)
                    x = 0 + self.car_position_offset[i] + j * self.car_position_interval[i] 
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    self.cars['init_poses'].append(pose)
                    self.cars['init_dir'].append(0)
                    count += 1
    
    def write2WorldFile(self):
        world_model = ''
        world_model += 'define block model\n' +\
            '(\n' +\
              'size [2.385 1.200 1.000]\n' +\
              'gui_nose 0\n' +\
            ')\n' +\
            'define topurg ranger\n' +\
            '(\n' +\
              'sensor(\n' +\
              'range [0.0 80.0]\n' +\
              'fov 180\n' +\
              'samples 360\n' +\
              ')\n' +\
              '# generic model properties\n' +\
              'color "black"\n' +\
              'size [ 0.050 0.050 0.100 ]\n' +\
            ')\n' +\
            'define gc1 position\n' +\
            '(\n' +\
              '#size [0.65 0.65 0.25]\n' +\
              '#origin [-0.05 0 0 0]\n' +\
              'size [2.385 1.200 1.000]\n' +\
              'origin [0.792 0.000 0.000 0.000]\n' +\
              'gui_nose 1\n' +\
              'drive "car"\n' +\
              'localization "gps"\n' +\
              '#odom_error [0.01 0.05 0.01 0.02 0.01 0.02]\n' +\
              'topurg(pose [1.985 0.000 -0.500 0.000])\n' +\
              'velocity_bounds [-10000 10000 -1 1 -1 1 -90 90 ]	\n' +\
              'acceleration_bounds [-10000 10000 -1 1 -1 1 -90 90]\n' +\
            ')\n' +\
            'define gc2 position\n' +\
            '(\n' +\
              '#size [0.65 0.65 0.25]\n' +\
              '#origin [-0.05 0 0 0]\n' +\
              'size [2.385 1.200 1.000]\n' +\
              'origin [0.792 0.000 0.000 0.000]\n' +\
              'gui_nose 1\n' +\
              'drive "car"\n' +\
              'localization "gps"\n' +\
              '#odom_error [0.01 0.05 0.01 0.02 0.01 0.02]\n' +\
              'topurg(pose [1.985 0.000 -0.500 0.000])\n' +\
              'velocity_bounds [-10000 10000 -1 1 -1 1 -90 90 ]	\n' +\
              'acceleration_bounds [-10000 10000 -1 1 -1 1 -90 90]\n' +\
            ')\n' +\
            'define floorplan model\n' +\
            '(\n' +\
              '# sombre, sensible, artistic\n' +\
              'color "gray30"\n' +\
              '# most maps will need a bounding box\n' +\
              'boundary 1\n' +\
              'gui_nose 0\n' +\
              'gui_grid 0\n' +\
            '#gui_movemask 0\n' +\
              'gui_outline 0\n' +\
              'gripper_return 0\n' +\
              'fiducial_return 0\n' +\
              'laser_return 1\n' +\
            ')\n' +\
            '# set the resolution of the underlying raytrace model in meters\n' +\
            'resolution 0.175438\n' +\
            'interval_sim 100  # simulation timestep in milliseconds\n' +\
            '#interval_real 100  # real-time interval between simulation updates in milliseconds \n' +\
            'window\n' +\
            '( \n' +\
              'size [ 357 572 ] \n' +\
              'center [143.242 97.300] \n' +\
            '#  center [-168.899 86.858]\n' +\
            '#rotate [ 0.000 -1.560 ]\n' +\
              'rotate [ 0.000 -407.500 ]\n' +\
              'scale 3.139 \n' +\
            ')\n' +\
            '# load an environment bitmap\n' +\
            'floorplan\n' +\
            '( \n' +\
              'name "willow"\n' +\
              'bitmap "cross_highway_collisioncheck_2.PNG"\n' +\
              'size [200.0 40.0 1.000]\n' +\
              'pose [100.0 20.0 0.000 0.000]\n' +\
            ')\n'
        world_model +=\
        '# throw in the autonomous car\n' +\
        'gc1( pose [100 5.0 0.000 90] name "gc1" color "red")\n'
        
        world_model += \
        '# throw in the agent cars\n'

        for i in range(len(self.cars['init_poses'])):
            init_pose = self.cars['init_poses'][i]
            pose = [init_pose[0], init_pose[1]]
            pose.append(0.0)
            pose.append(self.cars['init_dir'][i])
            name = self.cars['name'][i]
            color = 'green'
            robot_i = self.throwRobot(pose, name, color)
            world_model += robot_i


        # write world model to file
        f = open(self.path2world + self.world_name + '.world', 'w')
        f.write(world_model)
        # gc2( pose [3 6.5 0.000 0] name "gc2" color "green")

    def pose2Str(self, pose):
        return '[' + str(pose[0]) + ' ' + str(pose[1]) + ' ' + str(pose[2]) + ' ' + str(pose[3]) + ']'

    def throwRobot(self, pose, name, color):
        robot_str = 'gc2( pose '
        robot_str += self.pose2Str(pose) + ' name "' + name + '" color' +\
                ' "' + color + '")\n'
        return robot_str

if __name__=='__main__':
    if len(sys.argv) != 2:
        print 'Usage: python generate_world.py path2config'
        sys.exit(-1)

    worldGen = WorldGenerator(sys.argv[1])
