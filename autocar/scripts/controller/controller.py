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

class NewtonController:
    def __init__(self):
        self.init()
        self.start_game = False

    def init(self):

        self.hz = rospy.get_param('~hz', 10)
        self.max_speed = rospy.get_param('~max_speed', 5)
        self.min_speed = rospy.get_param('~min_speed', -5)
        self.rate = rospy.Rate(self.hz) 

        self.robot_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.acc = 0
        self.yaw = 0
        self.vel = 0

    def command_callback(self, msg):
        self.acc = msg.acc
        self.yaw = msg.yaw


    def speed_sign(self, msg):
        quaternion = msg.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        yaw = tf.transformations.euler_from_quaternion(explicit_quat)[2]
        yaw_car = np.array([cos(yaw), sin(yaw)])


        yaw_vel = atan2(msg.twist.twist.linear.y, msg.twist.twist.linear.x)
        yaw_vel = np.array([cos(yaw_vel), sin(yaw_vel)])

        if np.dot(yaw_car, yaw_vel) > 0:
            return 1
        else:
            return -1
        

    def print_odometry_msg(self, msg):
        quaternion = msg.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        print 'euler: ', tf.transformations.euler_from_quaternion(explicit_quat)
        print 'linear: ', msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z
        print 'vel yaw: ', atan2(msg.twist.twist.linear.y, msg.twist.twist.linear.x)
        print 'angular: ', msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z

    def state_callback(self, msg):
        a = 1
        # self.vel = self.speed_sign(msg) * sqrt(pow(msg.twist.twist.linear.x, 2) + pow(msg.twist.twist.linear.y, 2))
        
    def gameStarts(self, msg):
        if msg.state == 1:
            self.start_game = True

    def update_vel(self):
        self.vel = self.vel + self.acc * 1.0 / self.hz
        if self.vel > self.max_speed:
            self.vel = self.max_speed
        if self.vel < self.min_speed:
            self.vel = self.min_speed
        return self.vel

    def run(self):
        while not rospy.is_shutdown():
            if self.start_game:
                vel = self.update_vel()
                yaw = self.yaw
                
                self.send_control(vel, yaw)

            self.rate.sleep()
            
    def send_control(self, vel, yaw):
        msg = Twist()
        msg.linear.x = vel
        msg.angular.z = yaw
        self.robot_pub.publish(msg)

if __name__=='__main__':
    rospy.init_node('controller')

    controller = NewtonController()

    rospy.Subscriber('control_command', controlCommand, controller.command_callback, queue_size=1)
    rospy.Subscriber('base_pose_ground_truth', Odometry, controller.state_callback, queue_size=1)
    rospy.Subscriber('highway_game_start', RecordState, controller.gameStarts, queue_size=1)

    controller.run()
