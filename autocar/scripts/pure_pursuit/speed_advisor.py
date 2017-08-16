#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from geometry_msgs.msg import *
from math import *

class SpeedAdvisor:
    def __init__(self):
        self.hz = rospy.get_param('~hz', 5)
        self.speedType = rospy.get_param('~speed_type', 'constant')
        self.meanSpeed = rospy.get_param('~mean_speed', 1)
        self.speed_pub = rospy.Publisher('cmd_sm', TwistStamped, queue_size = 1)

    def run(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            speed = self.getSpeed()
            self.sendSpeed(speed)
            rate.sleep()

    def sendSpeed(self, speed):
        msg = TwistStamped()
        curTime = rospy.Time.now()
        msg.header.stamp = curTime
        msg.twist.linear.x = speed
        self.speed_pub.publish(msg)

    def constantSpeed(self):
        return self.meanSpeed

    def getSpeed(self):
        if self.speedType == 'constant':
            return self.constantSpeed()
        else:
            print 'unknown speed type!'
            sys.exit(-1)

if __name__=='__main__':
    rospy.init_node('speed_pub')

    speedPub = SpeedAdvisor()
    speedPub.run()
