import PySimpleGUI as sg   
import matplotlib.pyplot as plt
import matplotlib.image as mpimage   

def plot_pos(thrower_pos, goal_position, goalie_pos):
    fig = plt.figure(figsize=(15,15))
    ax1 = fig.add_subplot(111, aspect='equal')

    # Thrower
    ax1.plot(thrower_pos[0], thrower_pos[1], 'ch')
    ax1.annotate('Thrower', (thrower_pos[0], thrower_pos[1]), size=18)

    # Goalie
    ax1.plot(goalie_pos[0], goalie_pos[1], 'g^')
    ax1.annotate('Goalie', (goalie_pos[0], goalie_pos[1]), size=18)

    # Goal direction
    ax1.plot(goal_position[0], goal_position[1], 'ko')
    ax1.annotate('Goal', (goal_position[0], goal_position[1]), size=16)

    # Draw line
    p1 = [thrower_pos[0], goal_position[0]]
    p2 = [thrower_pos[1], goal_position[1]]
    ax1.plot(p1, p2, linestyle='dashed')

    ax1.axis([-5, 5, -5, 5])
    ax1.grid()

    # For the thrower area 
    rectangle = plt.Rectangle((0, -2.5), 3, 5, fc='blue',ec="red")
    plt.gca().add_patch(rectangle)

    # For the boundary 
    rectangle = plt.Rectangle((-3, -3.5), 7, 7, ec="black", facecolor='none')
    plt.gca().add_patch(rectangle)    

    # For the goal
    rectangle = plt.Rectangle((-4, -1), 1, 2, fc='red',ec="red")
    plt.gca().add_patch(rectangle)

    plt.show()
    return

def intersection_of_lines(line1, line2):
    """
    Parameters
    ----------
    line1: str
       Set thrower position
    line2: Environent
        Environment object created.
    
    Returns
    -------
    x, y in the goalee plane
    """
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(x, y):
        return x[0] * y[1] - x[1] * y[0]

    div = det(xdiff, ydiff)
    if div == 0:
       raise Exception('Lines do not intersect')

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div

    return [x, y]


def thrower_gui(env, thrower_id):

    """
    Parameters
    ----------
    identifier: str
       Set thrower position
    env: Environent
        Environment object created.
    thrower_id: int
        Thrower id, when multiple throwers.
    
    Returns
    -------
    None
    """

    layout = [  [sg.Text('Enter the x and y co-ordinates of the thrower', justification='center')],      
                [sg.Text('x: Range [0,3]                - ', size=(30, 1), key='-x-'), sg.InputText()],
                [sg.Text('y: Range [-2.5, 2.5]          - ', size=(30, 1), key='-y-'), sg.InputText()],
                [sg.Text('Goal position: Range (-0.8, 0.8) - ', size=(30, 1), key='-y-'), sg.InputText()],      
                [sg.Button('Show'), sg.OK(), sg.Button('Default')] ]    

    window = sg.Window('Thrower co-ordinates', layout)    

    while True:
        event, values = window.read()
        if values[0] is not '' and values[1] is not '' and values[2] is not '':
            values[0] = float(values[0])
            values[1] = float(values[1])
            values[2] = float(values[2])
        else:
            values[0] = 0
            values[1] = 0
            values[2] = 0

        if event in (sg.WIN_CLOSED, 'Default'):
            # From .g file
            values[0] = env.throwers[thrower_id].get_position()[0]
            values[1] = env.throwers[thrower_id].get_position()[1]
            values[2] = 0
            plot_pos([values[0], values[1]], [-3, values[2]], env.goalie.get_position())
            break

        if event in ('Show','OK'):
            if (values[0] > 3) or (values[0] < -0.5) or (values[1] > 2.5) or (values[1] < -2.5) or (values[2] > 0.8) or (values[2] < -0.8):  
                sg.popup(   'You entered x: ' + str(values[0]) + ' , y: ' + str(values[1]) + ' and goal position: ' + str(values[2]) + 
                            '. Invalid values entered. Accepted x: Range [0,3], y: Range [-2.5, 2.5] and goal position: Range [-1, 1]', 
                            title = 'Invalid inputs', text_color = 'red') 
            
            else:
                plot_pos([values[0], values[1]], [-3, values[2]], env.goalie.get_position())
                if event is 'OK':
                    break

    window.close()

    # print(event, values[0], values[1]) 
    return values[0], values[1], (intersection_of_lines( ((values[0], values[1]), (-3, values[2])), ((-2.5, -1), (-2.5, 1)) ))[1]