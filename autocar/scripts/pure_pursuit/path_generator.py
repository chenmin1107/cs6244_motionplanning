#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from math import *
import numpy as np


class PathGenerator:
    def __init__(self):
        self.path2file = rospy.get_param('~path2file', None)
        if self.path2file == None:
            print 'Please specify the path to the file that will store the path'
            sys.exit(-1)
        self.hz = rospy.get_param('~hz', 5)

        self.record_state = 0
        self.poses = []

    def robotPoseCallback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.record_state == 1:
            print '!! recording pose (record state): ', x, y, self.record_state
            self.poses.append([x, y])

    def recordStateCallback(self, msg):
        self.record_state = msg.state

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            if self.record_state == 2:
                print '!! finishes recording path!'
                self.write2File()
                break
            rate.sleep()

    def write2File(self):
        fpath = open(self.path2file, 'w')
        fpath.write(str(self.poses))
        fpath.close()

if __name__=='__main__':
    # if len(sys.argv) != 2:
        # print 'Usage: python path_generator.py robotID (e.g., robot_0)'
        # sys.exit(0)
    rospy.init_node('path_generator')
    
    pathGen = PathGenerator()
    
    rospy.Subscriber('base_pose_ground_truth', Odometry,\
            pathGen.robotPoseCallback, queue_size = 1)
    rospy.Subscriber('record_state', RecordState,\
            pathGen.recordStateCallback, queue_size = 1)
    pathGen.run()
