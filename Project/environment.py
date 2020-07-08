from settings import *
from thrower import *
from goalie import *

class Environment:
    def __init__(self, simulation, viewer, config, ry, time, np, math):
        self.simulation = simulation
        self.viewer     = viewer
        self.config     = config
        self.ry         = ry
        self.time       = time
        self.np         = np
        self.math       = math

        # Declare the thrower and goalie objects
        self.thrower = Thrower(self.simulation, self.viewer, self.config, self.ry, self.time, self.np, self.math)
        return

    # Outer wrapper for a member funciton call in thrower
    def thrower_is_gripper_grasping(self):
        return self.thrower.is_gripper_grasping()

    # Outer wrapper for a member funciton call in thrower
    def thrower_is_move_to_objective_fulfilled(self):
        return self.thrower.is_move_to_objective_fulfilled()

    # Outer wrapper for a member funciton call in thrower
    def thrower_set_move_to_objective(self, position):
        return self.thrower.set_move_to_objective(position)

    # Outer wrapper for a member funciton call in thrower
    def thrower_set_grab_objective(self, should_grab=False):
        return self.thrower.set_grab_objective(should_grab)

    # Outer wrapper for a member funciton call in thrower
    def thrower_set_throw_objective(self, position):
        return self.thrower.set_throw_objective(position)

    # Execute a time step
    def execute_time_step(self, tau):

        if self.thrower.getvar_throw_objective() is not None:
            # Throw
            #[y, J] = self.config.evalFeature(self.ry.FS.position, [self.gripper_center_identifier])

            oldQ = self.simulation.get_q()

            komo = self.config.komo_path(1., 1, tau, True)
            komo.clearObjectives()

            direction = [-(self.thrower.getvar_throw_objective()[0]), -(self.thrower.getvar_throw_objective()[1])]
            direction = direction / self.np.linalg.norm(direction)
            orthogonal = self.np.cross([direction[0], direction[1], 0], [0, 0, 1])

            # For smooth q                       
            komo.add_qControlObjective(order=1, scale=1e0)

            # directional objectives
            komo.addObjective([], self.ry.FS.vectorX, [robot_link1_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])
            komo.addObjective([], self.ry.FS.vectorX, [gripper_center_identifier], self.ry.OT.eq, [1e1], target=orthogonal)

            if self.thrower.getvar_throw_state() == 1:
                # Prepare throw
                angle_2 = throw_initial_angle2
                angle_4 = throw_initial_angle4
                angle_6 = throw_initial_angle6
                # To prevent collisions
                komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e1])
                komo.addObjective([], self.ry.FS.vectorY, [robot_link2_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(direction, -angle_2))
                komo.addObjective([], self.ry.FS.vectorY, [robot_link4_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(-direction, angle_4))
                komo.addObjective([], self.ry.FS.vectorY, [robot_link6_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(direction, -angle_6))

            elif self.thrower.getvar_throw_state() == 2:
                # Execute throw
                if self.thrower.getvar_throw_index() < throw_max_iterations:
                    #start
                    angle_2 = throw_initial_angle2 * ((throw_max_iterations - self.thrower.getvar_throw_index()) / throw_max_iterations)
                    angle_4 = throw_initial_angle4 * ((throw_max_iterations - self.thrower.getvar_throw_index()) / throw_max_iterations)
                    angle_6 = throw_initial_angle6 * ((throw_max_iterations - self.thrower.getvar_throw_index()) / throw_max_iterations)
                    komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e1])
                    komo.addObjective([], self.ry.FS.vectorY, [robot_link2_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(direction, -angle_2))
                    komo.addObjective([], self.ry.FS.vectorY, [robot_link4_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(-direction, angle_4))
                    komo.addObjective([], self.ry.FS.vectorY, [robot_link6_identifier], self.ry.OT.eq, [1e1], target=self.thrower.rotate(direction, -angle_6))
                else:
                    # end
                    #komo.addObjective([], self.ry.FS.vectorY, [robot_link2_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])
                    #komo.addObjective([], self.ry.FS.vectorY, [robot_link4_identifier], self.ry.OT.eq, [1e1], target=[-direction[0], -direction[1], 0])
                    komo.addObjective([], self.ry.FS.vectorY, [robot_link6_identifier], self.ry.OT.eq, [1e1], target=[direction[0], direction[1], 0])

                if self.thrower.getvar_throw_index() == throw_open_gripper_index:
                    # Open gripper
                    self.simulation.openGripper(gripper_identifier)

                self.thrower.setvar_throw_index(self.thrower.getvar_throw_index() + 1)

            ## apply speed
            #komo.add_qControlObjective(order=1, scale=(1./throw_speed))
            ## prevent collisions
            #komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e4])
            ## minimize position diff
            #komo.addObjective([1.], self.ry.FS.position, [gripper_center_identifier], self.ry.OT.sos, [1e1], target=self.thrower.getvar_throw_objective())
            ## don't close/open gripper
            #komo.addObjective([], self.ry.FS.qItself, [finger1_identifier], self.ry.OT.eq, scale=[1e1], order=1)
            #if self.is_gripper_grasping():
            #    # align gripper for opening
            #    komo.addObjective([], self.ry.FS.vectorZ, [gripper_center_identifier], self.ry.OT.eq, [1e1], target=[0, 0, 1])

            komo.optimize()

            self.config.setFrameState(komo.getConfiguration(0))
            newQ = self.config.getJointState()

            if self.thrower.getvar_throw_state() == 1:
                q = throw_preparation_movement_percentage * newQ + (1 - throw_preparation_movement_percentage) * oldQ
                if self.np.linalg.norm(self.np.abs(oldQ - newQ)) < 1.5e-1:
                    self.thrower.setvar_throw_state(2)
            
            #elif self.thrower.getvar_throw_state() == 2 and self.simulation.getGripperIsGrasping("R_gripper") == False:
                #self.thrower.getvar_throw_state() = 3

            else:
                q = newQ

            self.viewer.recopyMeshes(self.config)
            self.viewer.setConfiguration(self.config)

            self.simulation.step(q, tau, self.ry.ControlMode.position)

            # Check if gripper should open
            #error = self.np.linalg.norm(y-self.thrower.getvar_move_to_objective())
            #if error < self.throw_proximity_error:
            #    self.simulation.openGripper(self.gripper_identifier)
            return
        
        elif self.thrower.getvar_grab_objective() is not None or self.thrower.getvar_move_to_objective() is not None:
            # Grab or move
            if self.thrower.getvar_grab_objective() is not None:
                # Grab
                if self.thrower.getvar_grab_objective():
                    self.simulation.closeGripper(gripper_identifier)
                else:
                    self.simulation.openGripper(gripper_identifier)

                if self.thrower.is_gripper_grasping() == self.thrower.getvar_grab_objective():
                    self.thrower.set_grab_objective(None)

            if self.thrower.getvar_move_to_objective() is not None:
                # Move_to
                komo = self.config.komo_path(1., 1, tau, True)
                komo.clearObjectives()
                # apply speed
                komo.add_qControlObjective(order=1, scale=(1./move_speed))
                # Maintain pose closer to initial pose
                # komo.addObjective([], feature=self.ry.FS.qItself, type=self.ry.OT.eq, scale=[1e1], order=1)
                # prevent collisions
                komo.addObjective([], self.ry.FS.accumulatedCollisions, [], self.ry.OT.ineq, [1e0])
                # minimize position diff
                komo.addObjective([1.], self.ry.FS.position, [gripper_center_identifier], self.ry.OT.sos, [1e2], target=self.thrower.getvar_move_to_objective())
                # make gripper parallel to z axis
                komo.addObjective([], self.ry.FS.vectorZ, [gripper_center_identifier], self.ry.OT.eq, [1e1], target=[0., 0., 1])
                # don't close/open gripper
                komo.addObjective([], self.ry.FS.qItself, [finger1_identifier], self.ry.OT.eq, scale=[1e1], order=1)

                komo.optimize()

                self.config.setFrameState(komo.getConfiguration(0))
                q = self.config.getJointState()

                self.viewer.recopyMeshes(self.config)
                self.viewer.setConfiguration(self.config)

                self.simulation.step(q, tau, self.ry.ControlMode.position)

                # Check if objective is fulfilled
                if self.thrower.is_move_to_objective_fulfilled():
                    self.thrower.setvar_move_to_objective(None)
        else:
            # Do nothing
            return