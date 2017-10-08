#!/usr/bin/env python

import  os  
import  sys  
import roslib; roslib.load_manifest('autocar')  
import rospy 
from autocar.msg import *
from nav_msgs.msg import *
from math import *
import numpy as np
import json

MAX_X = 200
MAX_Y = 40
LANE_WIDTH = 3

class LaunchGenerator:
    def __init__(self, path2config):
        self.path2config = path2config
        
        self.loadConfig()
        self.write2LaunchFile()

    def loadConfig(self):
        print self.path2config
        with open(self.path2config, 'r') as f:
            self.d = json.load(f)

        self.world_name = self.d['world_name']
        self.num_lanes = self.d['num_lanes']
        self.path2launch = self.d['path_launch']
        self.num_cars_per_lane = self.d['num_cars_per_lane']

    def write2LaunchFile(self):
        launch_file = ''
        launch_file +=\
            '<launch>\n' +\
              '<param name="use_sim_time" value="true"/>\n' +\
              '<node name="map_server" pkg="map_server" type="map_server" args="$(find autocar)/maps/world/highway.yaml">\n'+\
              '  <param name="frame_id" value="/map" />\n'+\
              '</node>\n'

        launch_file +=\
          '<node name="stageros" pkg="stage_ros" type="stageros" args="-g $(find autocar)/maps/world/' + self.world_name + '.world" respawn="false" output="screen">\n'+\
          '  <param name="base_watchdog_timeout" value="0.2"/>\n'+\
          '</node>\n'

        # autonomous car
        launch_file += self.robotLaunch(0)

        # agent cars
        count = 1
        for i in range(self.num_lanes):
            num_cars_lane = self.num_cars_per_lane[i]
            for j in range(num_cars_lane):
                launch_file += self.robotLaunch(count)
                count += 1


        # launch_file += \
        # '<node pkg="autocar" type="highway_telop.py" name="highway_teleop_0" respawn="false" output="screen" >\n' +\
        # '<param name="hz" type="int" value="10" />\n' +\
        # '<param name="acc" type="double" value="5" />\n' +\
        # '<param name="yaw" type="double" value="0.0" />\n' +\
        # '</node>\n'

        launch_file += \
        '<!-- teleop control the robot -->\n' +\
        '<node pkg="autocar" type="controller.py" name="controller_0" respawn="false" output="screen" ns="/robot_0" >\n' +\
        '<param name="hz" type="int" value="10" />\n' +\
        '<param name="max_speed" type="double" value="5" />\n' +\
        '<param name="min_speed" type="double" value="-5" />\n' +\
        '</node>\n' +\
        '<node pkg="autocar" type="teleop.py" name="teleop_0" respawn="false" output="screen" ns="/robot_0" >\n' +\
        '<param name="hz" type="int" value="20" />\n' +\
        '<param name="acc" type="double" value="1" />\n' +\
        '<param name="yaw" type="double" value="0.25" />\n' +\
        '</node>\n'

        # speed controller for the agent cars
        launch_file +=\
        '<node pkg="autocar" type="highway_controller.py" name="high_controller" respawn="false" output="screen" >\n' +\
        '<param name="hz" type="int" value="10" />\n' +\
        '<param name="path2config" type="str" value="$(find autocar)/scripts/highway/' + self.world_name + '.json' + '" />\n' +\
        '</node>\n'

        # display
        launch_file += \
        '    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find autocar)/maps/rviz_settings/highway_simu.rviz"/>\n' +\
        '</launch>'

        # write world model to file
        f = open(self.path2launch + self.world_name + '.launch', 'w')
        f.write(launch_file)

    def robotLaunch(self, robot_id):
        robot_name = 'greenCar'
        if robot_id == 0:
            robot_name = 'redCar'
        robot_launch_str = ''
        robot_launch_str +=\
        '<node name="fake_localization_' + str(robot_id) + '" pkg="fake_localization" type="fake_localization" respawn="false" ns="/robot_'+ str(robot_id) + '">\n' +\
        '    <param name="odom_frame_id" value="/robot_' + str(robot_id) + '/odom"/>\n' +\
        '    <param name="base_frame_id" value="/robot_' + str(robot_id) + '/base_link"/>\n' +\
        '</node>\n' +\
        '<group ns="robot_' + str(robot_id) + '">\n' +\
        '    <include file="$(find autocar)/urdf/display_' + robot_name + '.launch"/>\n' +\
        '    <param name="tf_prefix" value="robot_' + str(robot_id) + '" />\n' + \
        '</group>\n'

        return robot_launch_str
    
if __name__=='__main__':
    if len(sys.argv) != 2:
        print 'Usage: python generate_world.py path2config'
        sys.exit(-1)

    launchGen = LaunchGenerator(sys.argv[1])
