# Thrower and Goalie Robot Arms
## Machine Learning and Robotics Lab, University of Stuttgart

## Team:
* Mohammed Muddasser
* Raphael Brösamle

## Introduction:
The project topic is to build a scenario consisting of a Thrower and a Goalie robot arms. The thrower moves to the desired position and throws the ball while the task of the goalie is to intercept and block the incoming ball from making a goal. 

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/Intro.gif" />
</p>
                                                                                                                   
The scenario is simple and synonymous to a goalkeeper in hockey, football or imitation of a block by a defender in basketball. The scenario is executed in an in-house simulation environment called Robotic AI (RAI) [https://github.com/MarcToussaint/rai] which uses the NVIDIA based PhysX simulation engine. RAI implements inverse kinematics using k-order Markov Path Optimization (KOMO)). The mesh model of a standard panda arm is is used for the thrower and goalie. The thrower is equipped with a gripper and the goalie is equipped with a flat paddle to its last joint. A suitable region behind the goalie is demarcated (goal) which the robot should prevent the ball from entering. 

The ball is thrown towards the goal by the thrower from random positions as set by the user. The trajectories are planned/directed towards random points on the goal plane.The thrower and goalie have a field of view of 180 degrees in front of them. A ball is removed from simulation once it has stopped moving for a certain amount of duration. Perception pipeline for ball detection and position estimation can be integarted into the environment.

This real time implementation of the setup has applications in crowd pulling, fun and entertainment activities. Also, it can be used for spot training in football, hockey or basketball practise and training. With a ring or cup shaped attachment it can also be extended for the game of throw and catch.

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/Simulation.gif" />
</p>
                                                                                                                   
## Setup:
This setup assumes that Ubuntu 18.04 is used.

Please first follow the instruction in the *Setup for Robotics Practical in Simulation* section of the [robotics-course](https://github.com/MarcToussaint/robotics-course#setup-for-robotics-practical-in-simulation) repository.

Install the gui and tkinter dependencies:
* `pip3 install gui`
* `sudo apt-get install python3-tk`

Please move all files and folders in the `scenarios` folder into the `../robotics_course/scenarios` folder.

Please also move the `meshes`folder into the `../robotics_course/scenarios` folder.

Please move all files in this repository into the `../robotics_course/SOME_FOLDER/SOME_OTHER_FOLDER` folder.

You can now run the jupyter notebook in this folder.

If you want more than one thrower in the simulation, you need to make some changes to the code.
Please open the `../robotics_course/scenarios/setup.g` file and uncomment the lines which are marked with MARK. Also in the `../robotics_course/SOME_FOLDER/SOME_OTHER_FOLDER/environment.py` file you need to change the `thrower_identifiers` variable, which is also marked with MARK, to `[1, 2]`. If you want to have just one thrower again, just revert all these steps.

## Sample Jupyter Notebooks and Options to be used 
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
* `loop`: If set to `True` the thrower will throw balls infinitely. If set to `False` the thrower will throw one ball and then stop

## Work Packages and File Structure
The code can be structured into following work packages.

### Work Package 1: Scene creation
The scenario as shown in figure below. Robot arm modifications to include the paddle and gripper accordingly. Add a thrower, goalie, goal and set the field area in the simulation environment.

Folder:

    Project\textbackslash meshes - contains goal mesh.
    
    Project\textbackslash scenarios - contains the graph data structure and .g files for the thrower and goalie.
    
Files:

    environment.py - main file for the project.
    
    state.py - state machine.

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/Setting.png" />
</p>

### Work Package 2: Thrower Trajectory Planning
A robot arm is used to throw balls at the target (as a ball cannon). Here, as a first step the robot arm is moved towards the ball, the ball is grasped, and lift is initiated. Inverse Kinematics is implemented using the standard RAI library functions. The force required is obtained by moving the robot arm to attain certain velocity and acceleration at the end effector and to open the gripper at the right moment to give it a desired trajectory. This setting is calibrated by trial and error and for desired minimal accuracy, as the ball does not need to hit the target at a specific point. Additional calibration to handle the ball slips from the robot gripper during the arm motion. 
The trajectory direction is estimated by calculating the line intersection between the thrower and goalie 2D positions. Refer Figure \ref{fig:trajectory}. \\

Files: In the folder 'Project'

    trajectory.py - trajectory planning
    
    thrower.py - thrower functionality

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/Trajectory.png" />
</p>

### Work Package 3: Ball Management
There are two modules for ball management.
At the thrower side it is used to keep track of the ball initiation and termination in the simulation environment. On the Goalie side it is used to calculate the trajectory and estimate the point of interception between the ball and the robot arm.

Files: In the folder 'Project'

    ballmanagement.py - Ball management

### Work Package 4: Goalie and Ball position estimation
Implemented robot manipulation using Inverse kinematics to implement a block. The position of the ball is estimated at every timestep of the simulation. The following two algorithms have been used and analysed (Refer Figure \ref{fig:trajectory}):
Trajectory based estimation:
    - Line intersection between the goal direction and the axis of the goal to get x and y co-ordinates of the intersection point.
    - 1D projectile motion to estimate the height of intersection(z).
Approximate estimation:
    - Line intersection between the goal direction and the axis of the goal to get x and y co-ordinates of the intersection point.
    - Track the height of the ball at every time step to get the height(z).

Files: In the folder 'Project'

    goalie.py - Goalie related functionality 

### Work Package 5: GUI
The GUI for setting the position of the thrower and position of the goal as shown in the figure below.

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/GUI.gif" />
</p>

Files:In the folder 'Project'

    gui.py - GUI related functionality

### Work Package 6: Example Jupyter Notebooks
A set of Jupyter Notebooks are added for the users to play around with the project. The Project.ipynb can be used as a starting point. The notebooks are self explanatory for the scenarios they demo.

Setup with two arms:

<p align="center">
<img src="https://github.com/muddasser27/Thrower_Goalie_RobotArms/blob/master/Video/Two_Arms.gif" />
</p>
