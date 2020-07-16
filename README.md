# Practical Course Robotics

### Repository - Throw and Block

## Team:
* Mohammed Muddasser
* Raphael Brösamle

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
You can chang some options to adjust the behavior of the throwers and the goalie in the notebooks.
Please refer to the provided Example notebooks for some examples.

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