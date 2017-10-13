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

class SpeedController:
    def __init__(self):
        self.init()
        self.start_game = False

    def init(self):
        self.path2config = rospy.get_param('~path2config', None)
        self.hz = rospy.get_param('~hz', 10)
        # self.path2config = 'Q1_world_1.json'
        if self.path2config == None:
            print 'path to config file is missing!'
            sys.exit(-1)

        self.rate = rospy.Rate(self.hz) 

        self.loadConfig()
        self.initPubs()

    def gameStarts(self, msg):
        if msg.state == 1 or msg.state == 2:
            self.start_game = True

    def initPubs(self):
        self.robot_pubs = {}
        count = 1
        for i in range(self.num_lanes):
            num_car_lane = self.num_cars_per_lane[i]
            for j in range(num_car_lane):
                self.robot_pubs[count] = {}
                self.robot_pubs[count]['laneid'] = i
                self.robot_pubs[count]['pub'] = rospy.Publisher('robot_' + str(count) + '/cmd_vel', Twist, queue_size=1)
                count += 1

    def loadConfig(self):
        with open(self.path2config, 'r') as f:
            self.d = json.load(f)

        self.world_name = self.d['world_name']
        self.num_lanes = self.d['num_lanes']
        self.num_cars_per_lane = self.d['num_cars_per_lane']
        self.mean_speed_per_lane = self.d['mean_speed_per_lane']
        self.stdrr_speed_per_lane = self.d['stdrr_speed_per_lane']

    def update_vel(self, i):
        mean_vel = self.mean_speed_per_lane[i]
        stdrr_vel = self.stdrr_speed_per_lane[i]
        return np.random.normal(mean_vel, stdrr_vel)

    def run(self):
        while not rospy.is_shutdown():
            # publish the speed for each robot
            if self.start_game:
                for i in self.robot_pubs:
                    vel = self.update_vel(self.robot_pubs[i]['laneid'])
                    yaw = 0
                    self.send_control(self.robot_pubs[i]['pub'], vel, yaw)
            self.rate.sleep()
            
    def send_control(self, robot_pub, vel, yaw):
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = 0
        robot_pub.publish(msg)

if __name__=='__main__':
    rospy.init_node('agent_controller')

    controller = SpeedController()
    rospy.Subscriber('/robot_0/highway_game_start', RecordState, controller.gameStarts, queue_size=1)

    controller.run()
