class Goalie:
    def __init__(self, simulation, viewer, config, ry):
        """
        Parameters
        ----------
        simulation:
            The simulation environment
        viewer:
            The viewer of the configuration
        config: libry.Config
            The config that should be used for manipulating the 
            simulation environment.
        ry: libry
            The libry module that was used for the config 
            parameter.
        """

        self.simulation = simulation
        self.viewer     = viewer
        self.config     = config
        self.ry         = ry

        self.move_to_objective = None
        self.direction_objective = None
        return

    paddle_identifier       = "Goalee_paddle"
    robot_base_identifier   = "Goalee_panda_link0"
    move_speed              = 1

    def get_position(self):
        """
        Returns
        -------
        array(x, y)
            Returns a 2D point containing the x and y coordinates
            of the goalies current position.
        """

        pos = self.config.frame(self.robot_base_identifier).getPosition()
        return [pos[0], pos[1]]

    def get_pad_position(self):
        """
        Returns
        -------
        array(x, y, z)
            Returns a 3D point containing the x, y and z coordinates
            of the goalies paddle.
        """

        pos = self.config.frame(self.paddle_identifier).getPosition()
        return pos

    def set_move_to_objective(self, position, move_speed = None):
        """
        Parameters
        ----------
        position: array(x, y, z)
            An array containing the 3D point to wich the goalie should
            move its paddle.
        move_speed: float
            A value describing how fast the goalie should move its 
            paddle to the specified position.
        """

        if move_speed is None:
            move_speed = self.move_speed        
        if position is None:
            self.move_to_objective = None
        elif len(position) != 3:
            print("Received position parameter with wrong length.")
            self.move_to_objective = None
        else:
            self.move_to_objective = [float(position[0]), float(position[1]), float(position[2])]
        return

    def set_direction_objective(self, direction):
        """
        Parameters
        ----------
        direction: array(x, y)
            A 2D point containing the x and y coordinates of the
            direction in which the goalie should point its paddle.
        """

        if direction is None:
            self.direction_objective = None
        elif len(direction) != 2:
            print("Received direction parameter with wrong length.")
            self.direction_objective = None
        else:
            self.direction_objective = [float(direction[0]), float(direction[1])]
        return

    def calculate_q_diff(self, tau):
        """
        Parameters
        ----------
        tau: float
            The time step that is used for the next simulation
            iteration.
        Returns
        -------
        q
            Depending of the objectives, this function will return
            the q differences that when applied will fulfill the objectives
            set with the setter functions.
        """
        
        oldQ = self.simulation.get_q()

        if self.move_to_objective is None and self.direction_objective is None:
            return oldQ - oldQ

        komo = self.config.komo_path(1., 1, tau, True)
        komo.clearObjectives()

        # For smooth q                       
        komo.add_qControlObjective(order=1, scale=(self.move_speed) * 1e0)
        # prevent collisions
        komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e3])

        if self.move_to_objective is not None:
            # Apply move_to objective.
            position = self.move_to_objective
            if position[2] < 0.1:
                position[2] = 0.1
            komo.addObjective([], self.ry.FS.position, [self.paddle_identifier], self.ry.OT.eq, [1e2], target=position)
        if self.direction_objective is not None:
            # Apply direction objective.
            direction = self.direction_objective
            komo.addObjective([], self.ry.FS.vectorZ, [self.paddle_identifier], self.ry.OT.eq, [1e1], target=[-direction[0], -direction[1], 0])
            komo.addObjective([], self.ry.FS.vectorX, [self.paddle_identifier], self.ry.OT.eq, [1e1], target=[0, 0, 1])

        komo.optimize()

        self.config.setFrameState(komo.getConfiguration(0))
        q = self.config.getJointState()

        return q - oldQ
