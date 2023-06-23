#Receives values from Epuck and plots them in function to position x-y, angle and distance from walls or objects

# Import
import serial
import struct
import sys
import signal
import time
from threading import Thread

import math
import pygame
import numpy as np
import matplotlib.pyplot as plt
import pygame.font


goodbye = """
          |\      _,,,---,,_
          /,`.-'`'    -.  ;-;;,_
         |,4-  ) )-,_..;\ (  `'-'
 _______'---''(_/--'__`-'\_)______   ______            _______  _
(  ____ \(  ___  )(  ___  )(  __  \ (  ___ \ |\     /|(  ____ \| |
| (    \/| (   ) || (   ) || (  \  )| (   ) )( \   / )| (    \/| |
| |      | |   | || |   | || |   ) || (__/ /  \ (_) / | (__    | |
| | ____ | |   | || |   | || |   | ||  __ (    \   /  |  __)   | |
| | \_  )| |   | || |   | || |   ) || (  \ \    ) (   | (      |_|
| (___) || (___) || (___) || (__/  )| )___) )   | |   | (____/\ _ 
(_______)(_______)(_______)(______/ |______/    \_/   (_______/(_)                                         
"""

# Initialisation de Pygame
pygame.init()

size_x = 1200
size_y = 600


# Définition de la taille de l'écran
screen = pygame.display.set_mode((size_x, size_y))
pygame.display.set_caption("2D Map")


#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)


#reads the FFT in float32 from the serial
def readFloatSerial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timeout...')
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size*4)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == 4*size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<f',rcv_buffer, i*4))
            i = i+1

        #print('received !')
        return data
    else:
        print('Timeout...')
        return []


#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)


#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = False
        self.contSendAndReceive = False
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))
        
        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    #function called after the init
    def run(self):
        while(self.alive):
            #flush the serial
            self.port.read(self.port.inWaiting())
            time.sleep(0.1)



    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()


#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()


# Function to reset the screen
def reset_screen():
    # Set the background color to black
    screen.fill((0, 0, 0))

    # Reset other variables as needed
    # ...

# Define button dimensions
button_width = 100
button_height = 40
# Create button surface
button_surface = pygame.Surface((button_width, button_height))
button_surface.fill((255, 255, 255))  # White color for the button
# Create button rectangle
button_rect = button_surface.get_rect()
button_rect.center = (800, 550)  # Position the button at (400, 550) on the screen


# Define the font for the button text
button_font = pygame.font.Font(None, 24)
# Create a surface for the button text
text_surface = button_font.render("Reset", True, (0, 0, 0))  # Black color for the text
# Get the rectangle of the text surface
text_rect = text_surface.get_rect()
text_rect.center = button_rect.center  # Position the text in the center of the button



# Boucle principale
running = True
while running:
    
    # Gestion des événements
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False  # Set the running flag to False to exit the loop
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left mouse button
                mouse_pos = pygame.mouse.get_pos()
                if button_rect.collidepoint(mouse_pos):
                    reset_screen()

    data = readFloatSerial(reader_thd.port)

    angle = 0
    distance = 0
    
    if(len(data)>0):
        
        # Extract float values from tuples
        float_data = [float(val[0]) for val in data]
        #print(float_data)
        x = float_data[0]
        y = float_data[1]
        angle = (360 - float_data[2]) - 90
        distance = float_data[3]
        # Change the Epuck 
        E_pos_x = (size_x/2) - 270/2 + x
        E_pos_y = (size_y/2) - 155/2 + y
        print(E_pos_x,E_pos_y)
 
        # Calcul des coordonnées du nouveau point
        radians = math.radians(angle)
        dx = distance * math.cos(radians)
        dy = distance * math.sin(radians)
        
        Point_x = E_pos_x + dx
        Point_y = E_pos_y + dy

        # Affichage du point à l'écran
        pygame.draw.circle(screen, (0, 255, 0), (E_pos_x,E_pos_y), 6)
        pygame.draw.circle(screen, (255, 0, 0), (Point_x,Point_y), 2)

    # Blit the button surface onto the screen surface
    screen.blit(button_surface, button_rect)

    # Blit the text surface onto the button surface
    screen.blit(text_surface, text_rect)

    pygame.display.update()

# Quit the Pygame and Python
reader_thd.stop()
pygame.quit()
print(goodbye)
quit()
