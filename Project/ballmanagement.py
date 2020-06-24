class BallManagement:
    """
    A Class used to manage the balls in the simulation environment.

    ...

    Methods
    -------
    create_ball(identifier, x, y, z)
        Creates a ball for the specified identifier and adds it 
        to the simulation environment. Return a corresponding
        Ball object.
    delete_ball(identifier)
        Delets the ball that matches the specified identifier and 
        removes it from the simulation environment. If no ball 
        matches the given identifier then nothing happens.
    get_ball(identifier)
        Returns a Ball object for the specified identifier. If no 
        ball matches the given identifier None is returned.
    """

    def __init__(self, config, ry):
        """
        Parameters
        ----------
        config: libry.Config
            The config that should be used for manipulating the 
            simulation environment.
        ry: libry
            The libry module that was used for the config 
            parameter.
        """

        self.config = config
        self.ry = ry
        self.dict = {}

    def create_ball(self, identifier, x, y, z):
        """
        Parameters
        ----------
        identifier: str
            A string that uniquely identifies the newly created ball.
        x: float
            The x position where the ball should be created.
        y: float
            The y position where the ball should be created.
        z: float
            The z position where the ball should be created.
        
        Returns
        -------
        Ball
            A corresponding Ball object for the newly created ball.
       """

        identifier = str(identifier)
        new_obj = self.config.addFrame(identifier)
        new_obj.setColor([1., 1., 0.])
        new_obj.setShape(self.ry.ST.sphere, [0.04])
        #new_obj.setShape(self.ry.ST.capsule, [0.05, 0.05])
        new_obj.setPosition([x, y, z])
        new_obj.setMass(0.1)
        new_obj.setContact(1)

        ball = Ball(self.config, identifier)
        self.dict[identifier] = ball
        return ball

    def delete_ball(self, identifier):
        """
        Parameters
        ----------
        identifier: str
            The identifier of the ball that should be deleted.
        """

        identifier = str(identifier)
        if identifier in self.dict:
            self.config.delFrame(identifier)
            self.dict.pop(identifier)
        return

    def get_ball(self, identifier):
        """
        Parameters
        ----------
        identifier: str
            The identifier for which a Ball object should be 
            returned.
        
        Returns
        -------
        Ball
            A corresponding Ball object for the specified 
            identifier. If no ball matches the given identifier 
            then None is returned.
        """

        identifier = str(identifier)
        if identifier in self.dict:
            return self.dict.get(identifier)
        else:
            print("The Ball with the identifier'" + identifier + "' doesn't exist.")
            return None

class Ball:
    """
    A Class representing the Balls in the simulation environment.

    ...

    Methods
    -------
    get_trajectory_estimate()
        TODO
    get_identifier()
        Returns the identifier of this Ball object.
    get_position()
        Returns the current position of this Ball in the simulation 
        environment.
    """

    def __init__(self, config, identifier):
        """
        Parameters
        ----------
        config: libry.Config
            The config that should be used for manipulating the 
            simulation environment.
        identifier: str
            The identifier of that uniquely identifies this ball.
        """

        self.config = config
        self.identifier = str(identifier)
        return

    def get_trajectory_estimate(self):
        """
        TODO
        """

        pass

    def get_identifier(self):
        """
        Returns
        -------
        str
            The identifier that uniquely identifies this object.
        """

        return self.identifier

    def get_position(self):
        """
        Returns
        -------
        array(x, y, z)
            An array containing the x, y, z coordinates of the 
            balls current position.
        """

        return self.config.getFrame(self.identifier).getPosition()