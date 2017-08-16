#!/usr/bin/env python

# Planner for autonomous vehicle Intersection navigation

# Author:
# Min CHEN

# SMART: Singapore MIT Alliance for Research and Technology

import  os  
import  sys  
import  tty, termios  
import roslib; 
# roslib.load_manifest('ipma')  
import rospy 
import pygame 
from pygame.locals import * 
from std_msgs.msg import String  

# global variables topic to be published
adAct_robot0 = advisedAction()
adAct_robot1 = advisedAction()
adAct_robot2 = advisedAction()
adAct_robot3 = advisedAction()
adAct_robot4 = advisedAction()
adActPub_robot0 = rospy.Publisher('/robot_0/advised_action', advisedAction, queue_size=1)
adActPub_robot1 = rospy.Publisher('/robot_1/advised_action', advisedAction, queue_size=1)
adActPub_robot2 = rospy.Publisher('/robot_2/advised_action', advisedAction, queue_size=1)
adActPub_robot3 = rospy.Publisher('/robot_3/advised_action', advisedAction, queue_size=1)
adActPub_robot4 = rospy.Publisher('/robot_4/advised_action', advisedAction, queue_size=1)

pygame.init()
clock = pygame.time.Clock()
screen = pygame.display.set_mode((150, 150))
  
def keyboardLoop():  
    # initialize
    rospy.init_node('AgActAdvisorTeleop')  
    rate = rospy.Rate(rospy.get_param('~hz', 10)) 
    action_pub_ = rospy.get_param('~action_pub', 1) 
  
    # show some message  
    print "Reading from keyboard" 
    print "Use keys to control the robots"    
    print "robot_0: w: accelerate, a: decelerate \n \
        robot_1: up: accelerate, down: decelerate\n\
        robot_2: r:accelerate, f: decelerate \n\
        robot_3: y:accelerate, h: decelerate \n\
        robot_4: pageup: accelerate, pagedown: decelerate"  
  
    # read the key   
    while not rospy.is_shutdown():       
        clock.tick(50) 
        keys = pygame.key.get_pressed()

        adAct_robot0.action = action_pub_
        adAct_robot1.action = action_pub_
        adAct_robot2.action = action_pub_
        adAct_robot3.action = action_pub_
        adAct_robot4.action = action_pub_

        # if keys[pygame.K_w]:
        #     print 'w has been pressed'
        # if keys[pygame.K_ESCAPE]:
        #     print('\nGame Shuting Down!')
        # if keys[pygame.K_UP]:
        #     print 'arrow up has been pressed' 
        for event in pygame.event.get():
            if event.type==pygame.QUIT:sys.exit()

        if(keys[pygame.K_w]):
            adAct_robot0.action = 0 
        elif(keys[pygame.K_s]):
            adAct_robot0.action = 2 
        if(keys[pygame.K_UP]):
            adAct_robot1.action = 0 
        elif(keys[pygame.K_DOWN]):
            adAct_robot1.action = 2 
        if(keys[pygame.K_r]):
            adAct_robot2.action = 0 
        elif(keys[pygame.K_f]):
            adAct_robot2.action = 2 
        if(keys[pygame.K_y]):
            adAct_robot3.action = 0 
        elif(keys[pygame.K_h]):
            adAct_robot3.action = 2 
        if(keys[pygame.K_PAGEUP]):
            adAct_robot4.action = 0 
        elif(keys[pygame.K_PAGEDOWN]):
            adAct_robot4.action = 2 


        # send message  
        adActPub_robot0.publish(adAct_robot0)
        adActPub_robot1.publish(adAct_robot1)
        adActPub_robot2.publish(adAct_robot2)
        adActPub_robot3.publish(adAct_robot3)
        adActPub_robot4.publish(adAct_robot4)  
        rate.sleep()  
  
if __name__ == '__main__':  
    try:  
        keyboardLoop()  
    except rospy.ROSInterruptException:  
        pass  
