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


class Teleop:
    def __init__(self):
        self.init()

    def init(self):
        pygame.init()
        clock = pygame.time.Clock()
        screen = pygame.display.set_mode((250, 250))

        rospy.init_node('highway_teleop')
        self.rate = rospy.Rate(rospy.get_param('~hz', 10)) 
        self.acc = rospy.get_param('~acc', 5)
        self.yaw = rospy.get_param('~yaw', 0)

        self.robot_pub = rospy.Publisher('robot_0/cmd_vel', Twist, queue_size=1)

        print "Usage: \n \
                up arrow: accelerate \n \
                down arrow: decelerate \n \
                left arrow: turn left \n \
                right arrow: turn right"

    def keyboard_loop(self):
        while not rospy.is_shutdown():
            acc = 0
            yaw = 0

            keys = pygame.key.get_pressed()
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    sys.exit()

            if(keys[pygame.K_UP]):
                acc = self.acc
            elif(keys[pygame.K_DOWN]):
                acc = -self.acc
            if(keys[pygame.K_LEFT]):
                yaw = self.yaw
            elif(keys[pygame.K_RIGHT]):
                yaw = -self.yaw

            self.send_control(acc, yaw)
            self.rate.sleep()  

    def send_control(self, vel, yaw):
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = yaw
        self.robot_pub.publish(msg)


if __name__=='__main__':
    teleop_agent = Teleop()
    teleop_agent.keyboard_loop()

