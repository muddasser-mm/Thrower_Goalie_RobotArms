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
    
    # Total number of throwers
    thrower_identifiers = [1]
    tau                 = 0.01
    throwers            = []
    states              = []
    states_index        = []
    goalie              = None
    thrower_movesteps   = 40 

    def start(self):
        self.config = self.ry.Config()

        self.config.addFile("../../scenarios/setup.g")

        self.viewer = self.ry.ConfigurationViewer()
        self.viewer.setConfiguration(self.config)

        self.viewer.recopyMeshes(self.config)
        self.viewer.setConfiguration(self.config)
        
        # To spawn a new ball at a random position (100 + int(thrower_identifier), 100, 0.5) TODO -Move ball creation in thrower at line 
        self.ball_management = BallManagement(self.config, self.ry, self.math)
        for thrower_identifier in self.thrower_identifiers:
            self.ball_management.create_ball("ball_" + str(thrower_identifier), 100 + int(thrower_identifier), 100, 0.5)

        # Start simulation engine
        self.simulation = self.config.simulation(self.ry.SimulatorEngine.physx, True)

        # Extract the inital poses of goalie and throwers
        self.q_init = self.simulation.get_q()

        # Create a goalie object
        self.goalie = Goalie(self.simulation, self.viewer, self.config, self.ry)

        # Create a thrower object
        self.count = 0
        for thrower_identifier in self.thrower_identifiers:
            thrower = Thrower(self.simulation, self.viewer, self.config, self.ry, self.time, self.np, self.math, thrower_identifier)
            self.throwers.append(thrower)
            ball = self.ball_management.get_ball("ball_" + str(thrower.identifier))
            thrower_pos = thrower.get_position()
            # Set ball spawn position
            ball.set_position([thrower_pos[0] + 0.5, thrower_pos[1], 0.1])
            self.count = self.count + 1

        # Create a state object and populate the current state of each thrower in list states_index
        for thrower in self.throwers:
            ball = self.ball_management.get_ball("ball_" + str(thrower.identifier))
            state = State(ball, thrower, self.goalie, self.np)
            self.states.append(state.get_states())
            self.states_index.append(0)

        # Update simulation view
        self.viewer.recopyMeshes(self.config)
        self.viewer.setConfiguration(self.config)
        self.simulation.setState(self.config.getFrameState())

        # To update the simulation view with the added ball. TODO - Not working when aligining to initial throw position
        self.simulation.step([], 0.01, self.ry.ControlMode.none)
        
        # Execute one throw and block
        self.throw_and_block()
    
    def throw_and_block(self):
        """
        Parameters
        ----------
        None.
        
        Returns
        -------
        None
            Executes one throw and block.
        """

        # Populate all the states and state indices for all throwers and call initialize()
        for i in range(len(self.states)):
            state = self.states[i]
            state_index = self.states_index[i]

            # Initiate Phase 1 : Picking up the ball
            print(state[state_index]["name"])
            state[state_index]["initialize"]()


        while True:
            q = self.simulation.get_q()

            # Get all balls in the simulation (Each thrower has one ball)
            self.balls = list(self.ball_management.dict.values())
            
            # Get all ball positions in balls_position
            self.balls_position = []
            for ball in self.balls:
                self.balls_position.append(ball.get_position())

            for i in range(len(self.states)):
                state = self.states[i]
                state_index = self.states_index[i]
                if len(state) > state_index:
                    state[state_index]["iterate"]()
                    if state[state_index]["is_done"]():
                        # Goto next phase. Excpet if in the last phase, then just increment self.states_index[i]
                        if len(state) > (state_index + 1):
                            print(state[state_index + 1]["name"])
                            state[state_index + 1]["initialize"]()
                        self.states_index[i] = state_index + 1
                else:
                    # Done
                    print("Done")
                    return
                # Compute new q of all throwers by using KOMO optimisation and update    
                q = q + self.throwers[i].calculate_q_diff(self.tau)
            
            # Update q of the goalie     
            q = q + self.goalie.calculate_q_diff(self.tau)

            # Update the simulation and configuration view
            self.viewer.recopyMeshes(self.config)
            self.viewer.setConfiguration(self.config)

            # Run one time step in simulation
            self.simulation.step(q, self.tau, self.ry.ControlMode.position)

            # Update ball velocities
            for i in range(len(self.balls)):
                ball = self.balls[i]
                prev_ball_pos = self.balls_position[i]
                ball.set_velocity(self.tau, prev_ball_pos)

    def move_thrower(self, thrower_identifier, position):
        """
        Parameters
        ----------
        thrower_identifier: Thrower id
        position: x, z coordinatesfor the thrower

        Returns
        -------
        None

            Moves the thrower to desired position
        """

        for step in range(self.thrower_movesteps):
            #x = (x1 + (fraction)(x2-x1))
            x = self.throwers[thrower_identifier].get_position()[0] + ((step + 1)/self.thrower_movesteps)*(position[0] - self.throwers[thrower_identifier].get_position()[0] )
            y = self.throwers[thrower_identifier].get_position()[1] + ((step + 1)/self.thrower_movesteps)*(position[1] - self.throwers[thrower_identifier].get_position()[1] )

            # Set new position of the thrower
            self.throwers[thrower_identifier].set_position([x, y, 0])
            
            # Update simulation view
            self.viewer.recopyMeshes(self.config)
            self.viewer.setConfiguration(self.config)
            self.simulation.setState(self.config.getFrameState())

            # To update the simulation view with the added ball. TODO - Not working when aligining to initial throw position
            self.simulation.step([], 0.01, self.ry.ControlMode.none)