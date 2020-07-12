import PySimpleGUI as sg   
import matplotlib.pyplot as plt
import matplotlib.image as mpimage   

def plot_pos(thrower_pos, goalie_pos):
    fig = plt.figure(figsize=(15,15))
    ax1 = fig.add_subplot(111, aspect='equal')

    # Thrower
    ax1.plot(thrower_pos[0], thrower_pos[1], 'ro')
    ax1.annotate('Thrower', (thrower_pos[0], thrower_pos[1]), size=18)

    # Goalie
    ax1.plot(goalie_pos[0], goalie_pos[1], 'ro')
    ax1.annotate('Goalie', (goalie_pos[0], goalie_pos[1]), size=18)

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
                [sg.Text('x: Range [0,3]        - ', size=(30, 1), key='-x-'), sg.InputText()],
                [sg.Text('y: Range [-2.5, 2.5]  - ', size=(30, 1), key='-y-'), sg.InputText()],     
                [sg.Button('Show'), sg.OK(), sg.Cancel()] ]    

    window = sg.Window('Thrower co-ordinates', layout)    

    while True:
        event, values = window.read()
        values[0] = float(values[0])
        values[1] = float(values[1])

        if event in (sg.WIN_CLOSED, 'Default'):
            # From .g file
            values[0] = env.throwers[thrower_id].get_position()[0]
            values[1] = env.throwers[thrower_id].get_position()[1]
            break

        if event in ('Show','OK'):
            if (values[0] > 3) or (values[0] < 0) or (values[1] > 2.5) or (values[1] < -2.5):  
                sg.popup(   'You entered x: ' + str(values[0]) + ' and y: ' + str(values[1]) + '. Invalid values entered. Accepted x: Range [0,3] and y: Range [-2.5, 2.5]', 
                            title = 'Invalid inputs', text_color = 'red') 
            
            else:
                plot_pos([values[0], values[1]], env.goalie.get_position())
                if event is 'OK':
                    break

    window.close()

    print(event, values[0], values[1]) 
    return values[0], values[1]