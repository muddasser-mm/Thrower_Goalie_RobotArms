# Practical Course Robotics

### Repository - Throw and Block

## Team:
* Mohammed Muddasser
* Raphael Brösamle

## Introduction:
The project topic is to build a scenario consisting of a Thrower and a Goalie robot arms. The thrower moves to the desired position and throws the ball while the task of the goalie is to intercept and block the incoming ball from making a goal. 

The scenario is simple and synonymous to a goalkeeper in hockey, football or imitation of a block by a defender in basketball. The scenario is executed in an in-house simulation environment called Robotic AI (RAI) [https://github.com/MarcToussaint/rai] which uses the NVIDIA based PhysX simulation engine. RAI implements inverse kinematics using k-order Markov Path Optimization (KOMO)). The mesh model of a standard panda arm is is used for the thrower and goalie. The thrower is equipped with a gripper and the goalie is equipped with a flat paddle to its last joint. A suitable region behind the goalie is demarcated (goal) which the robot should prevent the ball from entering. 

The ball is thrown towards the goal by the thrower from random positions as set by the user. The trajectories are planned/directed towards random points on the goal plane and within the goal dimensions.The thrower and goalie have a field of view of 180 degrees in front of them. A ball is removed from simulation once it has stopped moving for a certain amount of duration. The balls are shot periodically with certain interval of time from different positions continuously, until the simulation end time is reached. The perception pipeline is skipped for the task implementation such that the current ball position is directly queried through the API available in the current RAI simulation environment for trajectory planning and ball intercept position calculations. 

This real time implementation of the setup has applications in crowd pulling, fun and entertainment activities. Also, it can be used for spot training in football, hockey or basketball practise and training. With a ring or cup shaped attachment it can also be extended for the game of throw and catch.

## Setup:
This setup assumes that Ubuntu 18.04 is used.

Please first follow the instruction in the *Setup for Robotics Practical in Simulation* section of the [robotics-course](https://github.com/MarcToussaint/robotics-course#setup-for-robotics-practical-in-simulation) repository.

Install the gui and tkinter dependencies:
* `pip3 install gui`
* `sudo apt-get install python3-tk`

Please move all files and folders in the `scenarios` folder into the `$HOME/git/robotics_course/scenarios` folder.

Please also move the `meshes`folder into the `$HOME/git/robotics_course/scenarios` folder.

Please move all files in this repository into the `$HOME/git/robotics_course/SOME_FOLDER/SOME_OTHER_FOLDER` folder.

You can now run the jupyter notebook in this folder.

If you want more than one thrower in the simulation, you need to make some changes to the code.
Please open the `$HOME/git/robotics_course/scenarios/setup.g` file and uncomment the lines which are marked with MARK. Also in the `$HOME/git/robotics_course/SOME_FOLDER/SOME_OTHER_FOLDER/environment.py` file you need to change the `thrower_identifiers` variable, which is also marked with MARK, to `[1, 2]`. If you want to have just one thrower again, just revert all these steps.

## Options
You can change some options to adjust the behavior of the throwers and the goalie in the notebooks.
Please refer to the provided Example notebooks for running the code.

The `options` parameter is a dictionary containing dictionaries for each thrower in the simulation. The throwers are identified by `"Thrower" + str(thrower_identifier)`.

For each thrower you can provide a dictionary with the following values:
* `get_thrower_position_using`: If set to `"gui"` the simulation will show the gui and ask you to input the position to which the goalie should move and an offset for the goal position to which the thrower will throw the ball. If set to `"random"` these variables will be chosen randomly. If set to `"values"` the variables in the `thrower_position_values` and the `goal_intersection_y_value` will be used.
* `thrower_position_values`: If set to `[x, y]`, where `x` and `y` are two floating point values, then the thrower will be moved to this position. This only takes effect if the `get_thrower_position_using` field is set to `values`.
* `goal_intersection_y_value`: If set to any floating point values, then the thrower throw the ball onto the goal with the specified offset. This only takes effect if the `get_thrower_position_using` field is set to `values`.
* `change_thrower_position_smoothly`: If set to `True` the thrower will be moved smoothly to its new position. If set to `False`the thrower will be moved to its new posiiton instantly.
* `delay_time_steps`: If set to any integer larger than 0 the simulation will be delayed by that may time steps.
* `delay_only_once`: If set to `True` the delay from the `delay_time_steps` will be applied only once at the beginning of the simulation. If set to `False` the delay will be applied every time the thrower will start a new throw. This will only take effect if the `loop` field is set to `True`.
* `algorithm`: If set to `1` the goalie will use the trajectory based estimation to block all balls from this thrower, if set to `2` the goalie will use the approximate estimation.
* `loop`: If set to `True` the thrower will throw balls infinitely. If set to `False` the thrower will throw one ball and then stop.
