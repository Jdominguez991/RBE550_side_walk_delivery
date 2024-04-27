# RBE550_side_walk_delivery
The main basis for this project like all the urdfs and general was from: https://github.com/rodriguesrenato/warehouse_robot_simulation
## Download Applications
Some python applications are required in order to use the sqlite db as well as use the libraries in order to convert euler and quaternion. The following cmd should be run top to repo directory.
```
bash grab_programs.sh 
```

To install required submodule use the following cmd somewhere in the repo
```
git submodule update --init --recursive 
```

## Starting the sim

To run the sim use catkin_make and then use the run.launch file
```
catkin_make && roslaunch robot_operation run.launch
```
When running the sim there are two things to note
1. ### Errors in the Terminal
    Due to some time issue with amcl and the move_base code there is a constant error 
    ```
    [ WARN] [1714227051.593514483, 806.180000000]: Could not transform the global plan to the frame of the controller
    [ERROR] [1714227053.090221276, 807.280000000]: Extrapolation Error: Lookup would require extrapolation 0.065000000s into the future.  Requested time 807.191000000 but the latest data is at time 807.126000000, when looking up transform from frame [odom] to frame [map]

    [ERROR] [1714227053.090299298, 807.280000000]: Global Frame: odom Plan Frame size 20: map
    ```
    This is an error that was not able to be resolved, the robot will still operate and follow the path though it has been noticed the robot every now and then will be a little off course of expected. 
2. ### Long time robot moving back and fourth
    Sometime during the start up of the sim there is a long start as the move_base code is waiting for amcl to lock onto a pose. This can take a good chunk of time to speed it up. Use the "2D Pose Estimate" button at the top of rviz and click and drag the position the robot is roughly in currently.