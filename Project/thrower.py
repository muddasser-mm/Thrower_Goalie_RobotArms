class Thrower:
    def __init__(self, simulation, viewer, config, ry, time, np, math, identifier):
        self.simulation = simulation
        self.viewer = viewer
        self.config = config
        self.ry = ry
        self.time = time
        self.np = np
        self.math = math

        self.move_to_objective = None
        self.grab_objective = None
        self.throw_objective = None

        self.identifier = identifier
        self.gripper_identifier = "Thrower" + str(identifier) + "_gripper"
        self.gripper_center_identifier = "Thrower" + str(identifier) + "_gripperCenter"
        self.finger1_identifier = "Thrower" + str(identifier) + "_finger1"
        self.robot_base_identifier = "Thrower" + str(identifier) + "_panda_link0"
        self.robot_link1_identifier = "Thrower" + str(identifier) + "_panda_link1_1"
        self.robot_link2_identifier = "Thrower" + str(identifier) + "_panda_link2_1"
        self.robot_link4_identifier = "Thrower" + str(identifier) + "_panda_link4_1"
        self.robot_link6_identifier = "Thrower" + str(identifier) + "_panda_link6_1"
        return

    move_speed = 1.2
    move_to_proximity_error = 1.5e-2
    throw_preparation_movement_percentage = 0.02
    throw_max_iterations = 20
    init_pose_max_iterations = 20
    throw_open_gripper_index = 12
    throw_initial_angle2 = 0.35
    throw_initial_angle4 = 0.67
    throw_initial_angle6 = 1.05

    def is_gripper_grasping(self):
        return self.simulation.getGripperIsGrasping(self.gripper_identifier)

    def is_move_to_objective_fulfilled(self):
        if self.move_to_objective is None:
            return True
         # Check if objective has been achieved.
        [y, J] = self.config.evalFeature(self.ry.FS.position, [self.gripper_center_identifier])
        error = self.np.linalg.norm(y-self.move_to_objective)
        return error < self.move_to_proximity_error

    def is_init_pose_reached(self):
        if self.init_pose_objective is None:
            return True

    def get_position(self):
        pos = self.config.frame(self.robot_base_identifier).getPosition()
        return [pos[0], pos[1]]

    def set_move_to_objective(self, position):
        #print("Thrower: set_move_to_objective")
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
        #print("Thrower: set_grab_objective")
        if should_grab:
            self.grab_objective = True
        else:
            self.grab_objective = False
        if self.throw_objective is not None:
            print("Overriding the throw objective...")
            self.throw_objective = None
        return

    def set_throw_objective(self, position):
        #print("Thrower: set_throw_objective")
        if position == None:
            self.throw_objective = None
        elif len(position) != 2:
            print("Received position parameter with wrong length.")
            self.throw_objective = None
        else:
            self.throw_objective = [float(position[0]), float(position[1])]
            self.throw_state = 1
            self.throw_index = 0
            self.throw_initial_state = None

        if self.move_to_objective is not None or self.grab_objective is not None:
            print("Overriding the move_to and grab objectives...")
            self.move_to_objective = None
            self.grab_objective = None
        return

    def rotate(self, vec, angle):
        angle = angle * self.math.pi * 2
        height = self.math.sin(angle)
        scale = self.math.cos(angle)
        xy = vec * self.np.linalg.norm(vec) * scale
        return [xy[0], xy[1], height]
    
    def calculate_q_diff(self, tau):
        #print("Thrower: calculate_q_diff")
        if self.throw_objective is not None:
            # Throw
            #[y, J] = self.config.evalFeature(self.ry.FS.position, [self.gripper_center_identifier])

            oldQ = self.simulation.get_q()

            komo = self.config.komo_path(1., 1, tau, True)
            komo.clearObjectives()

            direction = [-self.throw_objective[0], -self.throw_objective[1]]
            direction = direction / self.np.linalg.norm(direction)
            orthogonal = self.np.cross([direction[0], direction[1], 0], [0, 0, 1])

            # For smooth q                       
            #komo.add_qControlObjective(order=1, scale=1e0)

            # directional objectives
            komo.addObjective([], self.ry.FS.vectorX, [self.robot_link1_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])
            komo.addObjective([], self.ry.FS.vectorX, [self.gripper_center_identifier], self.ry.OT.eq, [1e1], target=orthogonal)
    
            if self.throw_state == 1:
                # Prepare throw
                    angle_2 = self.throw_initial_angle2
                    angle_4 = self.throw_initial_angle4
                    angle_6 = self.throw_initial_angle6
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link2_identifier], self.ry.OT.eq, [1e1], target=self.rotate(direction, -angle_2))
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link4_identifier], self.ry.OT.eq, [1e1], target=self.rotate(-direction, angle_4))
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link6_identifier], self.ry.OT.eq, [1e1], target=self.rotate(direction, -angle_6))
            elif self.throw_state == 2:
                # Execute throw
                if self.throw_index < self.throw_max_iterations:
                    #start
                    angle_2 = self.throw_initial_angle2 * ((self.throw_max_iterations - self.throw_index) / self.throw_max_iterations)
                    angle_4 = self.throw_initial_angle4 * ((self.throw_max_iterations - self.throw_index) / self.throw_max_iterations)
                    angle_6 = self.throw_initial_angle6 * ((self.throw_max_iterations - self.throw_index) / self.throw_max_iterations)
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link2_identifier], self.ry.OT.eq, [1e1], target=self.rotate(direction, -angle_2))
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link4_identifier], self.ry.OT.eq, [1e1], target=self.rotate(-direction, angle_4))
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link6_identifier], self.ry.OT.eq, [1e1], target=self.rotate(direction, -angle_6))
                else:
                    # end
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link2_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link4_identifier], self.ry.OT.eq, [1e1], target=[-direction[0], -direction[1], 0])
                    komo.addObjective([], self.ry.FS.vectorY, [self.robot_link6_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])

                if self.throw_index == self.throw_open_gripper_index:
                    # Open gripper
                    self.simulation.openGripper(self.gripper_identifier)

                self.throw_index = self.throw_index + 1

            ## apply speed
            #komo.add_qControlObjective(order=1, scale=(1./self.throw_speed))
            ## prevent collisions
            #komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e4])
            ## minimize position diff
            #komo.addObjective([1.], self.ry.FS.position, [self.gripper_center_identifier], self.ry.OT.sos, [1e1], target=self.throw_objective)
            ## don't close/open gripper
            #komo.addObjective([], self.ry.FS.qItself, [self.finger1_identifier], self.ry.OT.eq, scale=[1e1], order=1)
            #if self.is_gripper_grasping():
            #    # align gripper for opening
            #    komo.addObjective([], self.ry.FS.vectorZ, [self.gripper_center_identifier], self.ry.OT.eq, [1e1], target=[0, 0, 1])

            komo.optimize()

            self.config.setFrameState(komo.getConfiguration(0))
            newQ = self.config.getJointState()

            if self.throw_initial_state is None:
                self.throw_initial_state = newQ
            elif self.throw_state == 1:
                newQ = self.throw_initial_state

            if self.throw_state == 1:
                q = self.throw_preparation_movement_percentage * newQ + (1 - self.throw_preparation_movement_percentage) * oldQ
                if self.np.linalg.norm(self.np.abs(oldQ - newQ)) < 1.5e-1:
                    self.throw_state = 2
            else:
                q = newQ

            return q - oldQ
        
        elif self.grab_objective is not None or self.move_to_objective is not None:
            # Grab or move
            if self.grab_objective is not None:
                # Grab
                if self.grab_objective:
                    self.simulation.closeGripper(self.gripper_identifier)
                else:
                    self.simulation.openGripper(self.gripper_identifier)

                if self.is_gripper_grasping() == self.grab_objective:
                    self.grab_objective = None
            
            if self.move_to_objective is not None:
                # Move to objective
                oldQ = self.simulation.get_q()

                komo = self.config.komo_path(1., 1, tau, True)
                komo.clearObjectives()
                
                # apply speed
                komo.add_qControlObjective(order=1, scale=(1./self.move_speed))
                # prevent collisions
                komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e2])
                # minimize position diff
                komo.addObjective([1.], self.ry.FS.position, [self.gripper_center_identifier], self.ry.OT.sos, [1e2], target=self.move_to_objective)
                # make gripper parallel to z axis
                komo.addObjective([], self.ry.FS.vectorZ, [self.gripper_center_identifier], self.ry.OT.eq, [1e1], target=[0., 0., 1])
                # don't close/open gripper
                komo.addObjective([], self.ry.FS.qItself, [self.finger1_identifier], self.ry.OT.eq, scale=[1e1], order=1)

                komo.optimize()

                self.config.setFrameState(komo.getConfiguration(0))
                q = self.config.getJointState()

                # Check if objective is fulfilled
                if self.is_move_to_objective_fulfilled():
                    self.move_to_objective = None

                return q - oldQ
        
        else:
            # Do nothing
            oldQ = self.simulation.get_q()
            return oldQ - oldQ