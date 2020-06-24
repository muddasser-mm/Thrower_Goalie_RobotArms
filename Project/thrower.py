class Thrower:
    def __init__(self, simulation, viewer, config, ry, time, np):
        self.simulation = simulation
        self.viewer = viewer
        self.config = config
        self.ry = ry
        self.time = time
        self.np = np

        self.move_to_objective = None
        self.grab_objective = None
        self.throw_objective = None
        return

    gripper_identifier = "gripper"
    gripper_center_identifier = "gripperCenter"
    finger1_identifier = "finger1"
    robot_base_identifier = "panda_link0"
    move_speed = 1.
    proximity_error = 1.5e-2
    throw_end_position_distance = 1.

    def is_gripper_grasping(self):
        return self.simulation.getGripperIsGrasping(self.gripper_identifier)

    def is_move_to_objective_fulfilled(self):
        if self.move_to_objective is None:
            return True
         # Check if objective has been achieved.
        [y, J] = self.config.evalFeature(self.ry.FS.position, [self.gripper_center_identifier])
        error = self.np.abs(y-self.move_to_objective).max()
        return error < self.proximity_error


    def set_move_to_objective(self, position):
        if position is None:
            self.move_to_objective = None
        elif len(position) != 3:
            print("Received position parameter with wrong length.")
            self.move_to_objective = None
        else:
            self.move_to_objective = [float(position[0]), float(position[1]), float(position[2])]
        if self.throw_objective is not None:
            print("Overriding the throw objective...")
            self.throw_objective = None
        return

    def set_grab_objective(self, should_grab=False):
        if should_grab:
            self.grab_objective = True
        else:
            self.grab_objective = False
        if self.throw_objective is not None:
            print("Overriding the throw objective...")
            self.throw_objective = None
        return

    def set_throw_objective(self, height):
        if height == None:
            self.throw_objective = None
        else:
            [y_robot, J_robot] = self.config.evalFeature(self.ry.FS.position, [self.robot_base_identifier])
            [y_gripper, J_gripper] = self.config.evalFeature(self.ry.FS.position, [self.gripper_center_identifier])
            y_throw_direction = (y_robot - y_gripper) + y_robot
            y_throw_target = self.throw_end_position_distance * (y_throw_direction / self.np.linalg.norm(y_throw_direction))
            self.throw_objective = [y_throw_target[0], y_throw_target[1], float(height)]
        if self.throw_objective is not None:
            print("Overriding the move_to and grab objectives...")
            self.move_to_objective = None
            self.grab_objective = None
        return

    def execute_time_step(self, tau):
        if self.throw_objective is not None:
            # Throw
            # TODO
            return
        elif self.grab_objective is not None or self.move_to_objective is not None:
            # Grab or move
            if self.grab_objective is not None:
                # Grab
                self.simulation.closeGripper(self.gripper_identifier)
                self.grab_objective = None
            if self.move_to_objective is not None:
                # Move_to
                komo = self.config.komo_path(1., 1, tau, True)
                komo.clearObjectives()
                # apply speed
                komo.add_qControlObjective(order=1, scale=(1./self.move_speed))
                # prevent collisions
                komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e4])
                # minimize position diff
                komo.addObjective([1.], self.ry.FS.position, [self.gripper_center_identifier], self.ry.OT.sos, [1e1], target=self.move_to_objective)
                # make gripper parallel to z axis
                komo.addObjective([], self.ry.FS.vectorX, [self.gripper_center_identifier], self.ry.OT.eq, [1e1], target=[1, 0., 0])
                komo.addObjective([], self.ry.FS.vectorY, [self.gripper_center_identifier], self.ry.OT.eq, [1e1], target=[0., 1, 0])
                # don't close/open gripper
                komo.addObjective([], self.ry.FS.qItself, [self.finger1_identifier], self.ry.OT.eq, scale=[1e1], order=1)

                komo.optimize()

                self.config.setFrameState(komo.getConfiguration(0))
                q = self.config.getJointState()

                self.viewer.recopyMeshes(self.config)
                self.viewer.setConfiguration(self.config)

                self.simulation.step(q, tau, self.ry.ControlMode.position)
        else:
            # Do nothing
            return