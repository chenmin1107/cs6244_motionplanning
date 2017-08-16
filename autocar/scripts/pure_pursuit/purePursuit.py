#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from math import *

class PurePublisher:
    def __init__(self):
        self.msg = Twist()
        self.hz = rospy.get_param('~hz', 10)
        self.pure_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

    def steerCallback(self, msg):
        self.msg = msg

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.pure_pub.publish(self.msg)
            rate.sleep()

if __name__=='__main__':
    rospy.init_node('purepursuit_pub')

    purePub = PurePublisher()
    rospy.Subscriber('cmd_steer', Twist, purePub.steerCallback, queue_size = 1)
    
    purePub.run()
