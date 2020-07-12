import PySimpleGUI as sg   
import matplotlib.pyplot as plt
import matplotlib.image as mpimage   

def plot_pos():
    image = mpimage.imread("field.png")
    plt.imshow(image, extent=[-5, 5, -5, 5])
    return

"""
    plt.plot(x, y)
    plt.plot(x[0], y[0], 'og')
    plt.plot(x[-1], y[-1], 'ob')
    plt.show()

"""
def thrower_gui():
    layout = [  [sg.Text('Enter the x and y co-ordinates of the thrower', justification='center')],      
                [sg.Text('x: Range [0,3]        - ', size=(30, 1), key='-x-'), sg.InputText()],
                [sg.Text('y: Range [-2.5, 2.5]  - ', size=(30, 1), key='-y-'), sg.InputText()],     
                [sg.OK(), sg.Cancel()] ]    

    window = sg.Window('Thrower co-ordinates', layout)    

    while True:
        event, values = window.read()
        values[0] = float(values[0])
        values[1] = float(values[1])

        if event in (sg.WIN_CLOSED, 'Cancel'):
            break

        if (values[0] > 3) or (values[0] < 0) or (values[1] > 2.5) or (values[1] < -2.5):  
            sg.popup(   'You entered x: ' + str(values[0]) + ' and y: ' + str(values[1]) + '. Invalid values entered. Accepted x: Range [0,3] and y: Range [-2.5, 2.5]', 
                        title = 'Invalid inputs', text_color = 'red') 
        else:
            #plot_pos()
            break

    window.close()

    print(event, values[0], values[1]) 
    return values[0], values[1]