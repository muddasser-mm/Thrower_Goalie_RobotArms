class State:
    def __init__(self, ball, thrower, goalie, np, tau):
        self.ball = ball
        self.thrower = thrower
        self.goalie = goalie
        self.np = np
        self.tau = tau

        self.direction_objective_flag = False
        return

    def do_nothing(self):
        return
    
    def thrower_move_to_ball(self):
        ball_position = self.ball.get_position()
        self.thrower.set_move_to_objective(ball_position)
        return

    def thrower_close_gripper(self):
        self.thrower.set_grab_objective(True)
        return

    def thrower_move_opposite_the_goal(self):
        # Appropriate parameter values for recoil position - Fixed values got from trial and testing
        distance_to_thrower = 0.6
        height = 0.4

        goalie_pos = self.goalie.get_position()
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
        thrower_pos = self.thrower.get_position()

        direction = [goalie_pos[0] - thrower_pos[0], goalie_pos[1] - thrower_pos[1]]
        self.thrower.set_throw_objective(direction)
        return

    def thrower_gripper_open_position(self):
        if self.thrower.throw_state == 2:
            distance_to_goalie = 0.5

            goalie_pos = self.goalie.get_position()
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
        print(dist_to_ball)

        # No change in direction when the ball is near
        if(dist_to_ball < 0.15):
            self.direction_objective_flag = True

        if not self.direction_objective_flag:
            self.goalie.set_direction_objective(direction)

        return

    def thrower_reset_pose(self):
        self.thrower.set_reset_pose_objective()
        return

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

    state_name          = "name"
    state_initialize    = "initialize"
    state_iterate       = "iterate"
    state_is_done       = "is_done"

    def get_states(self):
        return [
            {
                # Phase 1 : Picking up the ball
                self.state_name: "Picking up the ball",
                self.state_initialize: self.do_nothing,
                self.state_iterate: self.thrower_move_to_ball,
                self.state_is_done: self.thrower_is_move_done
            },
            {
                # Phase 2 : Grasping up the ball
                self.state_name: "Grasping the ball",
                self.state_initialize: self.thrower_close_gripper,
                self.state_iterate: self.thrower_move_to_ball,
                self.state_is_done: self.thrower_is_grasping
            },
            {
                # Phase 3 : Lift up the ball
                self.state_name: "Lift the ball",
                self.state_initialize: self.do_nothing,
                self.state_iterate: self.thrower_move_opposite_the_goal,
                self.state_is_done: self.thrower_is_move_done
            },
            {
                # Phase 4 : Throwing up the ball
                self.state_name: "Throwing the ball",
                self.state_initialize: self.thrower_throw,
                self.state_iterate: self.do_nothing,
                self.state_is_done: self.thrower_is_not_grasping
            },
            {
                # Phase 5 : Stopping up the ball
                self.state_name: "Stopping the ball",
                self.state_initialize: self.do_nothing,
                self.state_iterate: self.goalie_stop_ball_algo1,
                self.state_is_done: self.goalie_is_ball_stopped
            },
			{
			    # Phase 6 : Resetting back to initial pose
			    self.state_name: "Resetting to initial pose",
			    self.state_initialize: self.thrower_reset_pose,
			    self.state_iterate: self.do_nothing,
			    self.state_is_done: self.is_pose_reached
			}
        ]