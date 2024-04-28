# RBE550_side_walk_delivery

## Original project
The main basis for this project like all the urdfs and general was from: https://github.com/rodriguesrenato/warehouse_robot_simulation

## System requirements
* ROS Noetic
* installed move_base_msgs if not install using 
    ```
    apt-get install -y ros-noetic-move-base-msgs
    ```
* Installed tf2_sensor_msgs if not install using
    ```
    apt-get install -y ros-noetic-tf2-sensor-msgs
    ```
* System was run on Python 3.8 and virtual env assumed to be the same
## Getting started
1. Clone the repository into a catkin workspace under the src folder
    ```
    git clone git@github.com:Jdominguez991/RBE550_side_walk_delivery.git
    ```
2. Load the sub modules
    ```
    git submodule update --init --recursive
    ```
3. Load the python virtual environment tools using assuming that python virtual environment is already installed
    1. cd into the python_venv folder and run the cmd 
        ```
        python3.8 -m venv env
        ```
    2. Activate the environment
        ```
        source env/bin/activate
        ```
    3. Load in python requirements
        ```
        pip install -r requirements.txt
        ```
4. Move up to the top of the workspace
4. Compile ROS workspace
    ```
    catkin_make
    ```
5. Source the devel setup file
    ```
    source devel/setup.bash
    ```
6. Run the launch file
    ```
    roslaunch robot_operation run.launch 
    ```

## Errors when running the sim
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

## File Stucture
```
└── 📁RBE550_side_walk_delivery
    └── README.md
    └── 📁ira_laser_tools --Submodule for the ira tools to merge laser scans
    └── 📁maps --Map of the environment
        └── map1.pgm
        └── map1.yaml
    └── 📁motion_controller --Path planner code
        └── CMakeLists.txt
        └── 📁launch -- Launch file to use for debugging with vscode ROS debugger
            └── debug_ros.launch
        └── 📁msg 
            └── point.msg --A msg to represent one point of the path
        └── 📁src
            └── 📁algo_functions
                └── algorithm_publisher_subscriber.py --used to subscribe of publish for the algorithms functions 
                └── algorithms.py --A* planner code to find path from point a to b
            └── path_planner_main.py -- test code to see if A* works
            └── planner_service.py -- Node to act as service provider to find a path
        └── 📁srv
            └── path.srv -- File to define service of a path
    └── 📁order_handler -- General code for product spawner, world_handler, order_handler, and config files for move_base
        └── CMakeLists.txt
        └── 📁config --Config files for rviz and move_base
            └── basic_config.rviz
            └── 📁nav
                └── costmap_common_params.yaml
                └── global_costmap_params.yaml
                └── local_costmap_params.yaml
                └── trajectory_planner.yaml
            └── robot_config.rviz
        └── 📁launch --basic launch
            └── amcl.launch
            └── debug_ros.launch
            └── robot_description.launch
            └── robot_spawner.launch
            └── world.launch
        └── 📁model --Models from the original project
        └── package.xml
        └── 📁src
            └── conf.yaml
            └── move_base.py
            └── order_handeler.py
            └── product_spawner.py
            └── world_handler.py
        └── 📁world -- Different world files for gazebo
    └── 📁pointcloud_to_lasercan --Python file to convert point cloud to laser scan to see simulated people
    └── 📁python_venv -- python env files
        └── requirements.txt
    └── 📁robot_operation --Code to operate the robot
        └── CMakeLists.txt
        └── 📁launch --Launch file to start all other launch files to show robot moving
            └── run.launch
        └── package.xml
        └── setup.py
        └── 📁src
            └── main_operation.py -state machine to move the robot
            └── 📁move_base
                └── __init__.py
                └── move_base.py
        └── 📁srv --service to receive order
            └── send_order.srv
```