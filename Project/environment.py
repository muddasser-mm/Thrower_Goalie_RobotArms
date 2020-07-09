from thrower import *
from ballmanagement import *
from goalie import *
from state import *

class Environment:
    def __init__(self, ry, math, np, time):
        self.ry = ry
        self.math = math
        self.np = np
        self.time = time
        return
    
    thrower_identifiers = [1]
    tau = 0.01

    def start(self):
        self.config = self.ry.Config()

        self.config.addFile("../../scenarios/setup.g")

        self.viewer = self.ry.ConfigurationViewer()
        self.viewer.setConfiguration(self.config)

        self.viewer.recopyMeshes(self.config)
        self.viewer.setConfiguration(self.config)

        self.ball_management = BallManagement(self.config, self.ry, self.math)
        for thrower_identifier in self.thrower_identifiers:
            self.ball_management.create_ball("ball_" + str(thrower_identifier), 100 + int(thrower_identifier), 100, 0.5)

        self.simulation = self.config.simulation(self.ry.SimulatorEngine.physx, True)
        
        self.throwers = []
        for thrower_identifier in self.thrower_identifiers:
            thrower = Thrower(self.simulation, self.viewer, self.config, self.ry, self.time, self.np, self.math, thrower_identifier)
            self.throwers.append(thrower)
            ball = self.ball_management.get_ball("ball_" + str(thrower.identifier))
            thrower_pos = thrower.get_position()
            ball.set_position([thrower_pos[0] + 0.2, thrower_pos[1] + 0.5, 0.5])

        self.goalie = Goalie(self.simulation, self.viewer, self.config, self.ry)

        self.states = []
        self.states_index = []
        for thrower in self.throwers:
            ball = self.ball_management.get_ball("ball_" + str(thrower.identifier))
            state = State(ball, thrower, self.goalie, self.np)
            self.states.append(state.get_states())
            self.states_index.append(0)
        
        self.viewer.recopyMeshes(self.config)
        self.viewer.setConfiguration(self.config)
        self.simulation.setState(self.config.getFrameState())
        self.simulation.step([], 0.01, self.ry.ControlMode.none)

        for i in range(len(self.states)):
            state = self.states[i]
            state_index = self.states_index[i]
            print(state[state_index]["name"])
            state[state_index]["initialize"]()
        while True:
            q = self.simulation.get_q()

            self.balls = list(self.ball_management.dict.values())
            self.balls_position = []
            for ball in self.balls:
                self.balls_position.append(ball.get_position())

            for i in range(len(self.states)):
                state = self.states[i]
                state_index = self.states_index[i]
                if len(state) > state_index:
                    state[state_index]["iterate"]()
                    if state[state_index]["is_done"]():
                        if len(state) > (state_index + 1):
                            print(state[state_index + 1]["name"])
                            state[state_index + 1]["initialize"]()
                        self.states_index[i] = state_index + 1
                else:
                    # Done
                    print("Done")
                    return
                q = q + self.throwers[i].calculate_q_diff(self.tau)
            q = q + self.goalie.calculate_q_diff(self.tau)

            self.viewer.recopyMeshes(self.config)
            self.viewer.setConfiguration(self.config)

            self.simulation.step(q, self.tau, self.ry.ControlMode.position)

            # Update ball velocities
            for i in range(len(self.balls)):
                ball = self.balls[i]
                prev_ball_pos = self.balls_position[i]
                ball.set_velocity(self.tau, prev_ball_pos)

        ############################################
        thrower = self.throwers[0]
        ball = self.ball_management.get_ball("ball_1")
        state = 1

        previous_pos = [0, 0, 0]
        while True:
            if state == 1 or state == 2:
                thrower.set_move_to_objective(ball.get_position())
                if state == 1 and thrower.is_move_to_objective_fulfilled():
                    state = 2
                    print("Done moving")
                    thrower.set_grab_objective(True)
                if state == 2 and thrower.is_gripper_grasping():
                    state = 3
                    print("Done grasping")
            if state == 3:
                pos = ball.get_position()
                thrower.set_move_to_objective([0, 0.5, .1])
                if thrower.is_move_to_objective_fulfilled():
                    state = 4
                    print("Done lifting")
                    thrower.set_throw_objective([0, -1])
            oldQ = self.simulation.get_q()
            q = thrower.calculate_q_diff(.01)

            self.viewer.recopyMeshes(self.config)
            self.viewer.setConfiguration(self.config)

            self.simulation.step(q + oldQ, 0.01, self.ry.ControlMode.position)