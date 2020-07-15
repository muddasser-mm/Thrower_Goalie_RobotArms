class State:
    def __init__(self, ball, thrower, goalie, np, random, gui, env, tau):
        self.ball = ball
        self.thrower = thrower
        self.goalie = goalie
        self.np = np
        self.random = random
        self.gui = gui
        self.env = env
        self.tau = tau

        self.direction_objective_flag = False
        return

    def do_nothing(self):
        return

    def delay_time_steps(self):
        self.delay_index = self.delay_index + 1
        return

    def move_thrower_to_initial_position(self):
        total_steps = int(1. / self.thrower_move_step_size)

        fraction = total_steps - self.thrower_move_step_index
        if fraction <= 0:
            fraction = 0
            self.thrower_move_step_index = total_steps

        if self.thrower_initial_position is None or len(self.thrower_initial_position) != 2:
            return

        fraction = 1. / float(fraction)

        current_position = self.thrower.get_position()
        new_position_x = current_position[0] + fraction * (self.thrower_initial_position[0] - current_position[0])
        new_position_y = current_position[1] + fraction * (self.thrower_initial_position[1] - current_position[1])
        self.thrower.set_position([new_position_x, new_position_y, 0])

        self.thrower_move_step_index = self.thrower_move_step_index + 1
        return

    def thrower_move_to_ball(self):
        ball_position = self.ball.get_position()
        self.thrower.set_move_to_objective(ball_position)
        return

    def thrower_move_above_the_ball(self):
        ball_position = self.ball.get_position()
        self.thrower.set_move_to_objective([ball_position[0], ball_position[1], ball_position[2] + 0.2])
        return

    def thrower_close_gripper(self):
        self.thrower.set_grab_objective(True)
        return

    def thrower_move_opposite_the_goal(self):
        # Appropriate parameter values for recoil position - Fixed values got from trial and testing
        distance_to_thrower = 0.6
        height = 0.4

        goalie_pos = self.goalie.get_position()
        goalie_pos[1] = goalie_pos[1] + self.thrower_initial_goal_position
        thrower_pos = self.thrower.get_position()

        # normal vector from goalie to thrower
        direction = [thrower_pos[0] - goalie_pos[0], thrower_pos[1] - goalie_pos[1]]
        direction = direction / self.np.linalg.norm(direction)

        # Move to the throw position. 
        pos = thrower_pos + (distance_to_thrower * direction)
        pos = [pos[0], pos[1], height]

        self.thrower.set_move_to_objective(pos)
        return

    def thrower_throw(self):
        goalie_pos = self.goalie.get_position()
        goalie_pos[1] = goalie_pos[1] + self.thrower_initial_goal_position
        thrower_pos = self.thrower.get_position()

        direction = [goalie_pos[0] - thrower_pos[0], goalie_pos[1] - thrower_pos[1]]

        max_iterations_offset = self.thrower.math.floor(self.np.linalg.norm(direction))
        self.thrower.throw_max_iterations = self.thrower.throw_max_iterations + max_iterations_offset

        self.thrower.set_throw_objective(direction)
        return

    def thrower_gripper_open_position(self):
        if self.thrower.throw_state == 2:
            distance_to_goalie = 0.5

            goalie_pos = self.goalie.get_position()
            goalie_pos[1] = goalie_pos[1] + self.thrower_initial_goal_position
            thrower_pos = self.thrower.get_position()

            direction = [thrower_pos[0] - goalie_pos[0], thrower_pos[1] - goalie_pos[1]]
            direction = direction / self.np.linalg.norm(direction)

            plane_pos = goalie_pos + (distance_to_goalie * direction)
            plane_dir = self.np.cross([direction[0], direction[1], 0], [0, 0, 1])
            plane_dir = [plane_dir[0], plane_dir[1]]

            traject     = self.ball.get_trajectory_estimate([plane_pos[0], plane_pos[1], plane_dir[0], plane_dir[1]])
            velocity    = self.ball.get_velocity()
            #self.ball.get_angular_vel(self.ball.get_identifier())
            print("Intersection at : ", traject, " and velocity is : ", velocity)

        return

    def goalie_stop_ball_algo1(self):
        distance_to_goalie = 0.5

        goalie_pos = self.goalie.get_position()
        thrower_pos = self.thrower.get_position()

        direction = [thrower_pos[0] - goalie_pos[0], thrower_pos[1] - goalie_pos[1]]
        direction = direction / self.np.linalg.norm(direction)

        plane_pos = goalie_pos + (distance_to_goalie * direction)
        plane_dir = self.np.cross([direction[0], direction[1], 0], [0, 0, 1])
        plane_dir = [plane_dir[0], plane_dir[1]]

        traject = self.ball.get_trajectory_estimate([plane_pos[0], plane_pos[1], plane_dir[0], plane_dir[1]])
        if traject is None:
            return
        self.goalie.set_move_to_objective(traject)
        self.goalie.set_direction_objective(direction)
        return

    def goalie_stop_ball_algo2(self):
        distance_from_goalie    = 0.75
        distance_to_goal        = 0.5
        goalie_pos = self.goalie.get_position()

        # Get the direction of ball and its linear velocity
        ball_pos = self.ball.get_position()
        direction = [-(self.ball.get_direction()[0]) , -(self.ball.get_direction()[1]) ]
        direction = direction / self.np.linalg.norm(direction)

        # Estimate position of the bat
        pad_pos = [ball_pos[0], (ball_pos[1] + (self.ball.get_velocity()[1] * self.tau)), (ball_pos[2] + (self.ball.get_velocity()[2] * self.tau))]

        # if x in not in vicitnity of the goal, then don't track x
        if ball_pos[0] < (goalie_pos[0] - distance_to_goal) or ball_pos[0] > (goalie_pos[0] + distance_from_goalie):
            pad_pos[0]  = goalie_pos[0] + distance_from_goalie

        # if x behind goalee
        if ball_pos[1] < -1:
             pad_pos[1] = -1  

        # if y beyond goal on positive y axis 
        if ball_pos[1] > 1:
             pad_pos[1]  = 1

        # if y beyond goal on positive y axis 
        if ball_pos[1] < -1:
             pad_pos[1] = -1    

        # z beyond the goal
        if ball_pos[2] > 1:
            pad_pos[2] = 1                 

        self.goalie.set_move_to_objective(pad_pos)

        # Distance between ball and goalie pad
        dist_to_ball = self.np.linalg.norm(self.goalie.get_pad_position() - ball_pos)
        #print(dist_to_ball)

        # No change in direction when the ball is near
        if(dist_to_ball < 0.15):
            self.direction_objective_flag = True

        if not self.direction_objective_flag:
            self.goalie.set_direction_objective(direction)

        return

    def thrower_reset_pose(self):
        self.thrower.set_reset_pose_objective()
        return

    def reset_thrower_position_randomly(self):
        x = self.random.uniform(0, 3)
        y = self.random.uniform(-2.5, 2.5)

        self.thrower_initial_position = [x, y]
        return

    def reset_ball_position(self):
        pos = self.thrower.get_position()
        self.ball.set_position([pos[0] + 0.5, pos[1], 0.1])

        self.thrower.viewer.recopyMeshes(self.thrower.config)
        self.thrower.viewer.setConfiguration(self.thrower.config)
        self.thrower.simulation.setState(self.thrower.config.getFrameState())
        return

    def update_thrower_position_and_goal(self):
        if self.thrower_position_method == "random":
            x = self.random.uniform(0, 3)
            y = self.random.uniform(-2.5, 2.5)
            goal_pos = self.random.uniform(-0.8, 0.8)
            self.thrower_initial_position = [x, y]
            self.thrower_initial_goal_position = goal_pos
        elif self.thrower_position_method != "values":
            posx, posy, goal_position = self.gui(self.env, 0)
            self.thrower_initial_position = [posx, posy]
            self.thrower_initial_goal_position = goal_position
        return


    def is_delay_finished(self):
        if self.delay_index > self.delay_max_index:
            if not self.delay_only_once:
                self.delay_index = 0
            return True
        return False

    def thrower_was_moved(self):
        total_steps = int(1. / self.thrower_move_step_size)
        if self.thrower_move_step_index + 1 >= total_steps:
            self.thrower_move_step_index = 0
            return True
        return False

    def thrower_is_move_done(self):
        return self.thrower.is_move_to_objective_fulfilled()

    def thrower_is_grasping(self):
        return self.thrower.is_gripper_grasping()

    def thrower_is_not_grasping(self):
        return not self.thrower.is_gripper_grasping()

    def goalie_is_ball_stopped(self):
        # If y co-ordinate of the ball is beyond the goal 
        if (self.ball.get_position()[1] > 5) or (self.ball.get_position()[1] < -5) or (self.ball.get_position()[0] < self.goalie.get_position()[0] - 0.3) or (self.ball.get_position()[0] > 5):
            return True

        # RMS values across all direction is less than 0.01, then ball asssumed to be stand-still
        if self.np.linalg.norm(self.ball.get_velocity()) < 1 :
            return True

        return False

    def is_pose_reached(self):
        return self.thrower.is_init_pose_reached()

    def return_false(self):
        return False

    def return_true(self):
        return True



    state_name          = "name"
    state_initialize    = "initialize"
    state_iterate       = "iterate"
    state_is_done       = "is_done"

    thrower_position_method = None
    thrower_initial_position = []
    thrower_initial_goal_position = 0.
    thrower_move_step_size = 1
    thrower_move_step_index = 0

    delay_max_index = 0
    delay_index = 0
    delay_only_once = True

    algorithm = 1

    should_loop = False

    def get_states(self, options=None):
        if options is not None:
            if options.get("get_thrower_position_using") is not None:
                self.thrower_position_method = options.get("get_thrower_position_using")
            if options.get("thrower_position_values") is not None:
                self.thrower_initial_position = options.get("thrower_position_values")
            if options.get("goal_intersection_y_value") is not None:
                self.thrower_initial_goal_position = float(options.get("goal_intersection_y_value"))
            if options.get("change_thrower_position_smoothly") is not None:
                if options.get("change_thrower_position_smoothly"):
                    self.thrower_move_step_size = 0.025
            if options.get("delay_time_steps") is not None:
                self.delay_max_index = float(options.get("delay_time_steps"))
            if options.get("delay_only_once") is not None:
                if not options.get("delay_only_once"):
                    self.delay_only_once = False
            if options.get("algorithm") is not None:
                self.algorithm = int(options.get("algorithm"))
            if options.get("loop") is not None:
                if options.get("loop"):
                    self.should_loop = True

        list_of_states = []
        list_of_states.append({
            self.state_name: "Update thrower position",
            self.state_initialize: self.update_thrower_position_and_goal,
            self.state_iterate: self.do_nothing,
            self.state_is_done: self.return_true
        })
        if self.delay_max_index > 0:
            list_of_states.append({
                self.state_name: "Waiting...",
                self.state_initialize: self.do_nothing,
                self.state_iterate: self.delay_time_steps,
                self.state_is_done: self.is_delay_finished
            })
        list_of_states.append({
            self.state_name: "Move thrower to initial position",
            self.state_initialize: self.do_nothing,
            self.state_iterate: self.move_thrower_to_initial_position,
            self.state_is_done: self.thrower_was_moved
        })
        list_of_states.append({
            self.state_name: "Reset ball position",
            self.state_initialize: self.reset_ball_position,
            self.state_iterate: self.do_nothing,
            self.state_is_done: self.return_true
        })
        list_of_states.append({
            self.state_name: "Move above the ball",
            self.state_initialize: self.do_nothing,
            self.state_iterate: self.thrower_move_above_the_ball,
            self.state_is_done: self.thrower_is_move_done
        })
        list_of_states.append({
            self.state_name: "Picking up the ball",
            self.state_initialize: self.do_nothing,
            self.state_iterate: self.thrower_move_to_ball,
            self.state_is_done: self.thrower_is_move_done
        })
        list_of_states.append({
            self.state_name: "Grasping the ball",
            self.state_initialize: self.thrower_close_gripper,
            self.state_iterate: self.thrower_move_to_ball,
            self.state_is_done: self.thrower_is_grasping
        })
        list_of_states.append({
            self.state_name: "Lift the ball",
            self.state_initialize: self.do_nothing,
            self.state_iterate: self.thrower_move_opposite_the_goal,
            self.state_is_done: self.thrower_is_move_done
        })
        list_of_states.append({
            self.state_name: "Throwing the ball",
            self.state_initialize: self.thrower_throw,
            self.state_iterate: self.do_nothing,
            self.state_is_done: self.thrower_is_not_grasping
        })
        algo = self.goalie_stop_ball_algo1
        if self.algorithm == 2:
            algo = self.goalie_stop_ball_algo2
        list_of_states.append({
            self.state_name: "Stopping the ball",
            self.state_initialize: self.do_nothing,
            self.state_iterate: algo,
            self.state_is_done: self.goalie_is_ball_stopped
        })
        list_of_states.append({
		    self.state_name: "Resetting to initial pose",
		    self.state_initialize: self.thrower_reset_pose,
		    self.state_iterate: self.do_nothing,
		    self.state_is_done: self.is_pose_reached
		})
        if not self.should_loop:
            list_of_states.append({
                self.state_name: "Done",
                self.state_initialize: self.do_nothing,
                self.state_iterate: self.do_nothing,
                self.state_is_done: self.return_false
            })
        return list_of_states
