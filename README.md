# CS6244 Motion Planning and Control

Autonomous driving simulator for CS6244 course project

## Set up the Simulator
The simulator has only been tested under the following system settings. There is no guarantee that it will work in other variants.
To avoid unnessary difficulties, we suggest that you to follow the instructions below closely.

### 1. Install Ubuntu 14.04 (Recommended)

Install the recommended Ubuntu version on a clean partition of the hard disk. The following tutorial gives you an example on how to install Ubuntu along with Windows.

http://www.everydaylinuxuser.com/2015/11/how-to-install-ubuntu-linux-alongside.html
(This tutorial has been updated for Ubuntu 16.04, but the steps are the same for 14.04)


### 2. Install ROS indigo (only supported by Ubuntu 14.04 and Ubuntu 13.10)

#### Install
Note that our simulator runs in ROS indigo. Please make sure you choose the correct version.

The following link gives you detailed instructions on how to install ROS indigo on Ubuntu.
(Please choose "Desktop-Full Install")

http://wiki.ros.org/indigo/Installation/Ubuntu

#### Tutorials
If you are new to ROS, I suggest you go through a quick tutorial on the website
before you dig into the simulator.

http://wiki.ros.org/ROS/Tutorials#ROS_Tutorials

#### Rosbuild
Our simulator follows rosbuild, so please choose the right version when you go through 
the tutorial.

### 3. Clone the packges from this github repo

git clone https://github.com/chenmin1107/cs6244_motionplanning.git

### 4. Add the package path to ROS system path
Add your path to $ROS_PACKAGE_PATH, so the system can find it.
You can do it as follows:

* cd ~/

* vim .bashrc (or whatever editor you like)

* add the follwing line at the end of your .bashrc file

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:[your path to]/cs6244_motionplanning/

* source ~/.bashrc

### 5. Build the packages

When you build the packages, the compiler might tell you
[xxx] package can not be found. You can easily install them
by following the commands below:

1. sudo apt-get update

2. sudo apt-get install ros-indigo-[package name]

#### /autocar

This is the package you will work on. You need to

1. cd autocar/

2. rosmake

#### /mpav

This package provides some basic functionalities 
for the simulator, and you do not have worry about it. All you need to do is 

1. cd mpav/MPAVUtil/

2. rosmake


3. cd mpav/Steering_Control/

4. rosmake

### 6. Install pygame

sudo apt-get install python-pygame

### 7. Try a simple task

Now you are going to try a simple task of using keyboard to drive a car
in the simulator.

Steps to follow:

1. roslaunch autocar test.launch

2. make sure the small pygame window in on the top of your screen

3. Now you can drive the car in the simulator using the arrow keys. Congrats!

# Dynamic model of the car

Please refer to the following google doc.

https://docs.google.com/document/d/1dy3Zl7XDfvnEylQbOTgZ6XmN3IXKXdZUhXtyWUPQCkQ/edit?usp=sharing
