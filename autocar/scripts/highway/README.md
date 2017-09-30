# Autonomous driving for highway crossing

## How to run the highway task

roslaunch autocar dataXX.launch

where dataXX is the name of the data set we gave to you.

## Future positions of the agent cars

future_positions/dataXX_poses.json

File format of dataXX_poses.json:

key: 'robot_ID'

value: [[$x_0, y_0$, dt],
        [$x_1, y_1$, dt]]
