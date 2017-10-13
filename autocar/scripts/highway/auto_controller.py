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
from std_msgs.msg import String  
from autocar.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import *
from math import *
import tf
import numpy as np
from time import *
import json

class AutoController:
    def __init__(self, path2control):
        self.path2control = path2control
        self.init()

    def init(self):
        self.hz = 500
        self.rate = rospy.Rate(self.hz)

        self.robot_pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1)

        self.start_game = False
        # self.start_game = True
        # self.initTime = time() 
        # self.index = 0
        self.initUpdateVel = True
        self.TIME_PER_STEP = 0.1

        self.LoadControls()
        self.lenControls = len(self.controls)

    def LoadControls(self):
        with open(self.path2control, 'r') as f:
            self.controls = json.load(f)['robot_0']

    def gameStarts(self, msg):
        if msg.state == 2 and self.initUpdateVel:
            self.start_game = True
            self.initTime = time() 
            self.index = 0
            self.vel = 0
            if self.initUpdateVel:
                self.updateVel = True
                self.initUpdateVel = False

    def updatAccVel(self):
        acc = 0
        timeElapsed = time() - self.initTime
        # print 'time elapsed: ', timeElapsed, self.vel, self.updateVel
        if self.index <= self.lenControls - 1:
            acc = self.controls[self.index][0]
            if self.updateVel:
                # print timeElapsed, self.vel
                self.vel += acc * self.TIME_PER_STEP
                self.updateVel = False
            if timeElapsed > self.controls[self.index][1]:
                # print 'increase index: ', timeElapsed, self.controls[self.index][1]
                self.index += 1
                self.updateVel = True

        else:
            self.vel = 0

    def run(self):
        while not rospy.is_shutdown():
            if self.start_game:
                self.updatAccVel()
                ctr = Twist()
                ctr.linear.x = self.vel
                self.robot_pub.publish(ctr)

            sleep(0.0001)

if __name__=='__main__':
    if len(sys.argv) != 2:
        print 'Usage: python auto_controller.py path2control'
        sys.exit(-1)
    rospy.init_node('auto_controller')
    autoController = AutoController(sys.argv[1])

    rospy.Subscriber('/robot_0/highway_game_start', RecordState, autoController.gameStarts, queue_size=1)

    autoController.run()
