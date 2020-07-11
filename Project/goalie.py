class Goalie:
    def __init__(self, simulation, viewer, config, ry):
        self.simulation = simulation
        self.viewer = viewer
        self.config = config
        self.ry = ry

        self.move_to_objective = None
        self.direction_objective = None
        return

    paddle_identifier = "Goalee_paddle"
    robot_base_identifier = "Goalee_panda_link0"

    def get_position(self):
        pos = self.config.frame(self.robot_base_identifier).getPosition()
        return [pos[0], pos[1]]

    def set_move_to_objective(self, position):
        #print("Goalie: set_move_to_objective")
        if position is None:
            self.move_to_objective = None
        elif len(position) != 3:
            print("Received position parameter with wrong length.")
            self.move_to_objective = None
        else:
            self.move_to_objective = [float(position[0]), float(position[1]), float(position[2])]
        return

    def set_direction_objective(self, direction):
        #print("Goalie: set_direction_objective")
        if direction is None:
            self.direction_objective = None
        elif len(direction) != 2:
            print("Received direction parameter with wrong length.")
            self.direction_objective = None
        else:
            self.direction_objective = [float(direction[0]), float(direction[1])]
        return

    def calculate_q_diff(self, tau):
        #print("Goalie: calculate_q_diff")
        oldQ = self.simulation.get_q()

        if self.move_to_objective is None and self.direction_objective is None:
            return oldQ - oldQ

        komo = self.config.komo_path(1., 1, tau, True)
        komo.clearObjectives()

        # For smooth q                       
        komo.add_qControlObjective(order=1, scale=1e0)
        # prevent collisions
        komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e2])

        if self.move_to_objective is not None:
            # Apply move_to objective.
            position = self.move_to_objective
            komo.addObjective([], self.ry.FS.position, [self.paddle_identifier], self.ry.OT.eq, [1e2], target=position)
        if self.direction_objective is not None:
            # Apply direction objective.
            direction = self.direction_objective
            komo.addObjective([], self.ry.FS.vectorZ, [self.paddle_identifier], self.ry.OT.eq, [1e1], target=[-direction[0], -direction[1], 0])

        komo.optimize()

        self.config.setFrameState(komo.getConfiguration(0))
        q = self.config.getJointState()

        return q - oldQ
