def calculate_trajectory(ball, plane, math):
    """
    Parameters
    ----------
    ball: Ball
        The ball object for which the trajectory shall be calculated.
    plane: array(float)
        An array with 4 elements. The first two elements are the x and y coordinates of some point on the plane. 
        The third and the fourth element are the x and y are the x and y coordinates of the direction of the plane.
    math: math
        The math library.
    
    Returns
    -------
    array(float)
        An array with 3 elements containing the x, y and z coordinates of the calculated 3D position.
    """


    ####################
    # At first we calculate where the ball will hit the ground.
    # We use the quadratic function 'f(x) = a*x^2 + b*x + c' to estimate the trajectory of the Z axis only.

    g = 9.81 # The gravitational force
    g = -1 * g # The g force is pointing downwards
    vel = ball.get_velocity()
    pos = ball.get_position() - [0, 0, ball.get_radius()] # pos contains the lower most point of the ball. This point will touch the ground on impact.
    # g, vel and pos are the constants in the quadratic function mentioned above.

    # We use the quadratic formula (german: Mitternachtsformel) to calculate where f(x) = 0
    x = (vel[2] * vel[2])
    x = x - (4 * g * pos[2])
    if x < 0:
        return None
    x = math.sqrt(x)
    x = (-vel[2] - x) / (2 * g)
    
    if x < 0:
        # Something went wrong. x should be > 0.
        return None

    # Since we are disregarding air resistance and other forces (except the gravitational force) the velocity of the ball in the X and Y axies 
    # will never change.
    # We can therefore use 'x' to calculate the 3D point where the ball will hit the ground.
    zero_point = g * (x * x)
    zero_point = zero_point + (vel[2] * x)
    zero_point = zero_point + pos[2]
    zero_point = [(pos[0] + (vel[0] * x)), (pos[1] + (vel[1] * x)), zero_point]
    zero_point[2] = 0

    if plane is None:
        # If no plane is provided we will return the point where the ball will hit the ground.
        # We add the balls radius again and therefore return the center point of the ball.
        return [zero_point[0], zero_point[1], zero_point[2] + ball.get_radius()] 
        
    ####################
    # We will now compute the point where the ball will hit the plane.

    # We first look at the XY plane of the balls trajectory and the plane.
    # The trajectory and the plane are simplified as a line in the XY plane.
    # They can be mathematically expressed by a position on the line and the direction of the line.
    # This term will specify the trajectories line: '(traject_pos[0], traject_pos[1])^T + x*(traject_dir[0], traject_dir[1])^T
    traject_pos = [pos[0], pos[1]]
    traject_dir = [zero_point[0] - pos[0], zero_point[1] - pos[1]]
    # This term will specify the planes line: '(plane_pos[0], plane_pos[1])^T + y*(plane_dir[0], plane_dir[1])^T
    plane_pos = [plane[0], plane[1]]
    plane_dir = [plane[2], plane[3]]

    if (traject_dir[0] - plane_dir[0]) == (traject_dir[1] - plane_dir[1]):
        # Trajectory and plane are parallel
        return None

    # We will now calculate where the trajectory and the plane will intersect.
    # We now set the terms of the trajectory and the plane equal to each other and solve for x (not for y).
    divisor = traject_dir[1] * plane_dir[0]
    divisor = divisor - (traject_dir[0] * plane_dir[1])

    xx = (plane_pos[1] * plane_dir[0]) / divisor
    xx = xx - ((traject_pos[1] * plane_dir[0]) / divisor)
    xx = xx + ((traject_pos[0] * plane_dir[1]) / divisor)
    xx = xx - ((plane_pos[0] * plane_dir[1]) / divisor)

    if xx < 0:
        # Ball is already behind the plane
        # If 'xx < 0' then ball will travel away from the plane.
        return None
    
    # If we plug 'xx' into the trajectories term, we will get the point where the trajectory and the plane will intercect.
    # 'intercect_pos' is this point. (In 2D)
    intersect_pos = [traject_pos[0] + (xx * traject_dir[0]), traject_pos[1] + (xx * traject_dir[1])]

    if xx >= 1:
        # Intercection is behind zero_point
        # We simplify the trajectory and asume the ball will not bounce.
        # If we plug 1 into the trajectories term, we will get the 2D coordinates of the point where the ball will hit the ground.
        # Therefore any value larger than 1 will result in a point behind the plane.
        return [intersect_pos[0], intersect_pos[1], ball.get_radius()]
    
    # We now know the ball is flying towards the plane and will hit the plane without first touching the ground.
    # We already know where the ball will hit the plane in the X and Y axies.
    # We therefore calculate where the ball will hit the plane in the Z axis.
    # Since the ball travels with constant speed on the XY plane, we can get the height by the function mentioned in the beginning.
    # f(x * xx) will give us the height of the ball.
    xxx = xx * x
    intersect_height = g * (xxx * xxx)
    intersect_height = intersect_height + (vel[2] * xxx)
    intersect_height = intersect_height + pos[2]
    return [intersect_pos[0], intersect_pos[1], intersect_height + ball.get_radius()] # We add the balls radius back here.