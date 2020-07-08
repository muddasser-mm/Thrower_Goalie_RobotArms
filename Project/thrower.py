from settings import *

class Thrower:
    def __init__(self, p_simulation, p_viewer, p_config, p_ry, p_time, p_np, p_math):
        
        self.simulation = p_simulation
        self.viewer     = p_viewer
        self.config     = p_config
        self.ry         = p_ry
        self.time       = p_time
        self.np         = p_np
        self.math       = p_math

        # Read-Write thrower flags
        self.move_to_objective   = None
        self.grab_objective      = None
        self.throw_objective     = None
        self.throw_state         = None
        self.throw_index         = None

        return

    def is_gripper_grasping(self):
        return self.simulation.getGripperIsGrasping(gripper_identifier)

    def is_move_to_objective_fulfilled(self):
        if self.move_to_objective is None:
            return True
         # Check if objective has been achieved.
        [y, J] = self.config.evalFeature(self.ry.FS.position, [gripper_center_identifier])
        error = self.np.linalg.norm(y-self.move_to_objective)
        return error < move_to_proximity_error

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

    def set_throw_objective(self, position):
        if position == None:
            self.throw_objective = None
        elif len(position) != 2:
            print("Received position parameter with wrong length.")
            self.throw_objective = None
        else:
            self.throw_objective = [float(position[0]), float(position[1])]
            self.throw_state = 1
            self.throw_index = 0

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

    # Get the flags
    def getvar_move_to_objective(self):
        return self.move_to_objective

    def getvar_grab_objective(self):
        return self.grab_objective

    def getvar_throw_objective(self):
        return self.throw_objective

    def getvar_throw_state(self):
        return self.throw_state

    def getvar_throw_index(self):
        return self.throw_index

    # Set the flags
    def setvar_move_to_objective(self, p_move_objective):
        self.move_to_objective = p_move_objective
        return

    def setvar_grab_objective(self, p_grab_objective):
        self.grab_objective = p_grab_objective
        return

    def setvar_throw_objective(self, p_throw_objective):
        self.throw_objective = p_throw_objective
        return 

    def setvar_throw_state(self, p_throw_state):
        self.throw_state = p_throw_state
        return 

    def setvar_throw_index(self, p_throw_index):
        self.throw_index = p_throw_index
        return 