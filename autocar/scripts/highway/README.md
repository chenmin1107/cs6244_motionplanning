# Autonomous driving for highway crossing

## How to run the highway task

1. roslaunch autocar dataXX.launch (e.g., data1.launch)

where dataXX is the name of the data set we gave to you.

2. Put the pygame window on the top of your screen

3. Press 's' key to start the highway simulation task. 

(The autonomous car does not move by default, and your job is to write a motion planner to control the autonomous car)

## Subscribed topic (autonomous car)

The autonomous is subscribing the topic below:

1. topic name: /robot_0/control_command

2. msg type (controlCommand): 

    float32 acc

    float32 yaw

3. The controls computed by your motion planner should be sent to the topic above. 
Note that in the highway task, you are only controling the acceleration, the yaw should always be zero.

## Published topics

The following is a list of topics that the simulator is publishing:

/robot_i/base_pose_ground_truth (i = 1, 2, 3, ... , K)

where i indicates the ith agent car on road. Your motion planner should subscribe to the topic above to get the current state of the ith agent car.

msg type (nav_msgs/Odometry):

http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html


## Goal region

The autonomous car is considered to have crossed the road if its y-coordiate is larger than 27 

## Parameters

1. Car length: 2.385, car width: 1.2

2. Car local axis origin: (1.8, 0.6), see the figure below:

![alt text](figs/car_params.png)

3. Map size: (X * Y) = (200m * 40m)

4. Map origin (0, 0) is at the left-bottom corner

5. Lane width: 3m

## Input file: future positions of the agent cars

locations of input file: future_positions/dataXX_poses.json 

File format of dataXX_poses.json:

{'robot_1': [[x_0, y_0, t_0], 
              [x_1, y_1, t_1],
              ... 
              [x_N, y_N, t_N]]

 .
 .
 .

 'robot_K': [[x_0, y_0, t_0], 
              [x_1, y_1, t_1],
              ... 
              [x_N, y_N, t_N]]
}

where K is the number of agent cars on road. N is the length of the recorded data sequence.
x_i, y_i (i = 0, 1, ... , N) are the coordinates at the ith time step.
t_0, ... , t_N are the timestamps of the data sequence, and the simulation starts at t_0 = 0.

## Output file: the controls computed by your motion planner 

Output file: 'dataXX_controls.json' 

File format of dataXX_controls.json:

{'robot_0': [[acc_0, t_0], 
             [acc_1, t_1],
              ... 
             [acc_M, t_M]]
}

where M is the number of steps it takes for the autonomous car to cross the road. acc_i is the 
acceleration of the autonomous car at timestamp t_i.
