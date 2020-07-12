class State:
    def __init__(self, ball, thrower, goalie, np):
        self.ball = ball
        self.thrower = thrower
        self.goalie = goalie

        self.np = np
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
        distance_to_thrower = 0.6
        # Appropriate height chosen to recoil
        height = 0.3

        goalie_pos = self.goalie.get_position()
        thrower_pos = self.thrower.get_position()

        # normal vector from goalie to thrower
        direction = [thrower_pos[0] - goalie_pos[0], thrower_pos[1] - goalie_pos[1]]
        direction = direction / self.np.linalg.norm(direction)

        # Move to the throw position.
        # TODO - Will not work once we move to negative x-axis. We should limit the thrower position to positive only. 
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

    def goalie_stop_ball(self):
        distance_to_goalie = 0.4

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

    def thrower_reset_pose(self):
        # TODO- Remvove, use for testing
        #self.thrower.set_move_to_objective([2.4, -1.5, 1.5])
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
        #TODO -  Remove next line, used for debug
        #return self.thrower.is_move_to_objective_fulfilled()
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
                self.state_iterate: self.goalie_stop_ball,
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