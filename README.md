# cs6244_motionplanning

Autonomous driving simulator for cs6244 course project

## How to set up
(Note: the simulator has only been tested in the following system
setup. There is no guarantee it will work in other versions of systems.
To avoid any unecessary trouble, I suggest you to strictly follow the instructions
below.)

### 1. Install Ubuntu 14.04 (Recommended)

The following tutorial gives you an example on how to install Ubuntu along with Windows.

http://www.everydaylinuxuser.com/2015/11/how-to-install-ubuntu-linux-alongside.html

You can always Google to find more informations on this.

### 2. Install ROS indigo (Supported only by Ubuntu 14.04 and Ubuntu 13.10)

#### install
Note that our simulator runs in ROS indigo. Please make sure you choose the correct version.

The following link gives you detailed instructions on how to install ROS indigo on Ubuntu.
(Please choose "Desktop-Full Install")

http://wiki.ros.org/indigo/Installation/Ubuntu

#### tutorials
If you are new to ROS, I suggest you go through a quick tutorial on the website
before you dig into the simulator.

http://wiki.ros.org/ROS/Tutorials#ROS_Tutorials

#### rosbuild or catkin ?
Our simulator follows rosbuild, so choose the right version when you go through 
the tutorial.

### 3. Clone the packges from this github repo

git clone https://github.com/chenmin1107/cs6244_motionplanning.git

### 4. Add this package to ROS system path
Add your directory to this package to $ROS_PACKAGE_PATH, so the system can find it.
You can do it as follows:

cd ~/
vim .bashrc

add the follwing line at the end of your .bashrc file

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:<your path to>/cs6244_motionplanning/

Then,

source ~/.bashrc

### 5. List of content and build the packages

#### /autocar

This is the package you will work on. You need to

cd autocar/
rosmake

This will build the package for you.

#### /mpav

This package provides some basic functionalities 
for the simulator, and you do not have worry about it. All you need to do is 

cd mpav/MPAVUtil/
rosmake

cd mpav/Steering_Control/
rosmake


### 6. Simple task to try

Now you are going to try a simple task of using keyboard to drive a car
in the simulator.

Steps to follow:

1. roslaunch autocar test.launch

2. make sure the small pygame window in on the top of your screen

3. Now you can drive the car in the simulator using the arrow keys. Congrats!
