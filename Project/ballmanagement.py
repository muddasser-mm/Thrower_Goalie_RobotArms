class BallManagement:
    def __init__(self, config, ry):
        self.config = config
        self.ry = ry
        self.dict = {}

    def create_ball(self, identifier, x, y, z):
        identifier = str(identifier)
        new_obj = self.config.addFrame(identifier)
        new_obj.setColor([1., 1., 0.])
        new_obj.setShape(self.ry.ST.sphere, [0.05])
        #new_obj.setShape(self.ry.ST.capsule, [0.05, 0.05])
        new_obj.setPosition([x, y, z])
        new_obj.setMass(0.1)
        new_obj.setContact(1)

        ball = Ball(self.config, identifier)
        self.dict[identifier] = ball
        return ball

    def delete_ball(self, identifier):
        identifier = str(identifier)
        if identifier in self.dict:
            self.config.delFrame(identifier)
            self.dict.pop(identifier)
        return

    def get_ball(self, identifier):
        identifier = str(identifier)
        if identifier in self.dict:
            return self.dict.get(identifier)
        else:
            print("The Ball with the identifier'" + identifier + "' doesn't exist.")
            return None

class Ball:
    def __init__(self, config, identifier):
        self.config = config
        self.identifier = identifier
        return

    def get_trajectory_estimate(self):
        pass

    def get_identifier(self):
        return self.identifier

    def get_position(self):
        return self.config.getFrame(self.identifier).getPosition()