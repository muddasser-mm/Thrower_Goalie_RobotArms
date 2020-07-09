def calculate_trajectory(ball, plane, math):
    g = 9.81
    g = -1 * g
    vel = ball.get_velocity()
    pos = ball.get_position() - [0, 0, ball.get_radius()]

    x = (vel[2] * vel[2])
    x = x - (4 * g * pos[2])
    x = math.sqrt(x)
    x = (-vel[2] - x) / (2 * g)
    
    if x < 0:
        # Something went wrong. x should be > 0.
        return None

    zero_point = g * (x * x)
    zero_point = zero_point + (vel[2] * x)
    zero_point = zero_point + pos[2]
    zero_point = [(pos[0] + (vel[0] * x)), (pos[1] + (vel[1] * x)), zero_point]
    zero_point[2] = 0

    if plane is None:
        return [zero_point[0], zero_point[1], zero_point[2] + ball.get_radius()]

    traject_pos = [pos[0], pos[1]]
    traject_dir = [zero_point[0] - pos[0], zero_point[1] - pos[1]]
    plane_pos = [plane[0], plane[1]]
    plane_dir = [plane[2], plane[3]]
    #print("traject_pos:")
    #print(traject_pos)
    #print("traject_dir:")
    #print(traject_dir)
    #print("plane_pos:")
    #print(plane_pos)
    #print("plae_dir:")
    #print(plane_dir)

    if (traject_dir[0] - plane_dir[0]) == (traject_dir[1] - plane_dir[1]):
        # Trajectory and plane are parallel
        return None

    divisor = traject_dir[1] * plane_dir[0]
    divisor = divisor - (traject_dir[0] * plane_dir[1])

    xx = (plane_pos[1] * plane_dir[0]) / divisor
    xx = xx - ((traject_pos[1] * plane_dir[0]) / divisor)
    xx = xx + ((traject_pos[0] * plane_dir[1]) / divisor)
    xx = xx - ((plane_pos[0] * plane_dir[1]) / divisor)
    #print("xx:")
    #print(xx)

    if xx < 0:
        # Ball is already behind the plane
        return None
    
    intersect_pos = [traject_pos[0] + (xx * traject_dir[0]), traject_pos[1] + (xx * traject_dir[1])]

    if xx >= 1:
        # Intercection is behind zero_point
        return [intersect_pos[0], intersect_pos[1], ball.get_radius()]
    
    xxx = xx * x
    intersect_height = g * (xxx * xxx)
    intersect_height = intersect_height + (vel[2] * xxx)
    intersect_height = intersect_height + pos[2]
    return [intersect_pos[0], intersect_pos[1], intersect_height + ball.get_radius()]