#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from math import *
import ast

class PathPublisher:
    def __init__(self):
        self.hz = rospy.get_param('~hz', 5)
        self.path2file = rospy.get_param('~path2file', None)
        print self.path2file
        if self.path2file == None:
            print 'Please specify the path to the file that stores the path'
            sys.exit(-1)

        self.readPath()
        self.path_pub = rospy.Publisher('tracking_path', Path, queue_size = 1)
        
    def readPath(self):
        fpath = open(self.path2file, 'r')
        poses = ast.literal_eval(fpath.readline())
        curTime = rospy.Time.now()

        self.path = Path()
        self.path.header.stamp = curTime
        self.path.header.frame_id = '/map'
        for pose in poses:
            point = PoseStamped()
            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]
            self.path.poses.append(point)

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.path_pub.publish(self.path)
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('path_pub')

    pathPub = PathPublisher()
    pathPub.run()
