#!/usr/bin/env python

# Planner for autonomous vehicle Intersection navigation

# Author:
# Min CHEN

# cs6244: Robot Motion Planning

import  os  
import  sys  
import  tty, termios  
import roslib; roslib.load_manifest('autocar')  
import rospy 
import pygame 
from pygame.locals import * 
from std_msgs.msg import String  
from autocar.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import *
from math import *
import tf
import numpy as np
import json

DT = 0.1
DATA_T_LEN = 30

MAX_X = 200
MAX_Y = 40
LANE_WIDTH = 3

class FuturePositions:
    def __init__(self, path2config, path2data):
        self.path2config = path2config
        self.path2data = path2data

        self.loadConfig()
        self.generatePositions()
        self.dumpData()
    
    def loadConfig(self):
        with open(self.path2config, 'r') as f:
            self.d = json.load(f)

        self.world_name = self.d['world_name']
        self.num_lanes = self.d['num_lanes']
        self.bir_direction = self.d['bir_direction']
        self.num_cars_per_lane = self.d['num_cars_per_lane']
        self.mean_speed_per_lane = self.d['mean_speed_per_lane']
        self.stdrr_speed_per_lane = self.d['stdrr_speed_per_lane']
        self.car_position_offset = self.d['car_position_offset']
        self.bottom_space = self.d['bottom_space']
        self.car_position_interval = self.d['car_position_interval']

    def generatePoses(self, init_pose, speed, sign_dir, ori_car):
        poses = []
        num_points = int(DATA_T_LEN / DT)
        for k in range(num_points):
            newposex = init_pose[0] + sign_dir * DT * k * speed
            newposey = init_pose[1]
            newposet = round(DT * k, 2)
            poses.append([newposex, newposey, ori_car, newposet])
        return poses

    def generatePositions(self):
        self.positions = {}
        count = 1
        if self.bir_direction == 0:
            # all car driving to the left
            for i in range(self.num_lanes):
                num_car_lane = self.num_cars_per_lane[i]
                for j in range(num_car_lane):
                    x = MAX_X - self.car_position_offset[i]\
                            - j * self.car_position_interval[i]
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    speed = self.mean_speed_per_lane[i]
                    robot_name = 'robot_' + str(count)
                    self.positions[robot_name] = self.generatePoses(pose, speed, -1, 180)
                    count += 1

        if self.bir_direction == 1:
            # bottom 5 lanes driving to the left
            count = 1
            for i in range(self.num_lanes/2):
                num_cars_lane = self.num_cars_per_lane[i]
                for j in range(num_cars_lane):
                    x = MAX_X - self.car_position_offset[i]\
                            - j * self.car_position_interval[i]
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    speed = self.mean_speed_per_lane[i]
                    robot_name = 'robot_' + str(count)
                    self.positions[robot_name] = self.generatePoses(pose, speed, -1, 180)
                    count += 1
            for i in range(self.num_lanes/2, self.num_lanes):
                num_cars_lane = self.num_cars_per_lane[i]
                for j in range(num_cars_lane):
                    x = 0 + self.car_position_offset[i] + j * self.car_position_interval[i] 
                    y = self.bottom_space + i * LANE_WIDTH + LANE_WIDTH / 2.0
                    pose = (x, y)
                    speed = self.mean_speed_per_lane[i]
                    robot_name = 'robot_' + str(count)
                    self.positions[robot_name] = self.generatePoses(pose, speed, 1, 0)
                    count += 1

    def dumpData(self):
        with open(self.path2data, 'w') as f:
            json.dump(self.positions, f)

if __name__=='__main__':
    if len(sys.argv) != 3:
        print 'Usage: python generate_future_positions.py path2config path2data'
        sys.exit(-1)

    genPose = FuturePositions(sys.argv[1], sys.argv[2])
