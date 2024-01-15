from vpython import *       #imports everything from python
import numpy as np
import time
import serial

RAD_TO_DEG = 57.295779513082320876798154814105        #radians to degrees converter
DEG_TO_RAD = 1/57.295779513082320876798154814105      #degrees to radians converter

#Connecting to the correct serial port and at the correct baud rate
arduinoData=serial.Serial('com4',115200)
time.sleep(1)

#Setting the canvas
scene = canvas(range = 5, width = 800, height = 800, background = color.yellow, forward = vector(-0.15, -0.25, -0.25))

#All objects used in making the rocket
battery1 = box(length = 1.75, width = 0.45, height = 0.45, color = color.blue, pos = vector(-0.6, 0.3, 0), axis = vector(0, 0, 1))      #makes a battery
battery2 = box(length = 1.75, width = 0.45, height = 0.45, color = color.blue, pos = vector(0.6, 0.3, 0), axis = vector(0, 0, 1))       #makes a battery
stick1 = cylinder(length = 3, radius = 0.075, color = color.black, pos = vector(-1.6, -3, -1), axis = vector(0, 1, 0))                  #makes a stick 1
stick2 = cylinder(length = 3, radius = 0.075, color = color.black, pos = vector(-1.6, -3, 1), axis = vector(0, 1, 0))                   #makes a stick 2
stick3 = cylinder(length = 3, radius = 0.075, color = color.black, pos = vector(1.6, -3, 1), axis = vector(0, 1, 0))                    #makes a stick 3
stick4 = cylinder(length = 3, radius = 0.075, color = color.black, pos = vector(1.6, -3, -1), axis = vector(0, 1, 0))                   #makes a stick 4
pcb = box(length = 2.25, width = 2.25, height = 0.1, color = color.green, pos = vector(0, -0.55, 0))                                    #makes a pcb
edf = cylinder(length = 0.5, radius = 1.25, color = color.black, pos = vector(0, -3.25, 0), axis = vector(0, 1, 0))                     #makes an edf
cyl_battery = cylinder(length = 0.1, radius = 2, color = color.white, axis = vector(0, 1, 0))                                           #makes an up-right cylinder for batteries
cyl_pcb = cylinder(length = 0.1, radius = 2, color = color.white, pos = vector(0, -0.7, 0), axis = vector(0, 1, 0))                     #makes an up-right cylinder for pcb
cyl_edf = cylinder(length = 0.1, radius = 2, color = color.white, pos = vector(0, -3, 0), axis = vector(0, 1, 0))                       #makes an up-right cylinder for edf
rocket = compound([battery1, battery2, stick1, stick2, stick3, stick4, pcb, edf, cyl_battery, cyl_pcb, cyl_edf])
rocket.opacity = 0.4

#All objects used in making the vectors. These can be treated as unit vectors.
#Orignal frame of reference
xarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.red, pos = vector(0, -0.5, 0), axis=vector(1,0,0))             #arrow in x-direction
yarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.green, pos = vector(0, -0.5, 0), axis=vector(0,1,0))           #arrow in y-direction
zarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.blue, pos = vector(0, -0.5, 0), axis=vector(0,0,1))            #arrow in z-direction
#Constantly moving frame of reference
new_xarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.purple, pos = vector(0, -0.5, 0), axis=vector(1,0,0))      #new arrow in x-direction
new_yarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.magenta, pos = vector(0, -0.5, 0), axis=vector(0,1,0))     #new arrow in y-direction
new_zarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.orange, pos = vector(0, -0.5, 0), axis=vector(0,0,1))      #new arrow in z-direction

#Angles for simulation
while (True):
    q0, q1, q2, q3 = 0, 0, 0, 0     #please change these values after input from arduino
    pitch, roll, yaw = 0, 0, 0

    #Recieving data from serial port
    while (arduinoData.inWaiting() == 0):
        pass
    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')       #Removing the /b and other such formating of string
    splitPacket = dataPacket.split(",")         #Splits the string into multiple strings, by splitting it at the comma
    pitch = -float(splitPacket[0])              #Coverting first string to float as pitch
    roll = float(splitPacket[1])                #Coverting second string to float as roll
    yaw = -float(splitPacket[2])                #Coverting third string to float as yaw
    print(f'{pitch}, {roll}, {yaw}')            #Just to make sure!

    """
    #Converting quaternions to euler angles
    pitch = -np.pi/2 + 2*atan2((1 + 2*(q0*q2 - q1*q3))**0.5, (1 - 2*(q0*q2 - q1*q3))**0.5)
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2))
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2**2 + q3**2))
    """

    #Maths for finding new x, y, z axis after motion
    rate(50)
    new_x = vector(cos(pitch)*cos(yaw), sin(pitch), cos(pitch)*sin(yaw))    #for further info check notes
    int_y = vector(0, 1, 0)         #intermediate step 1; check notes for explaination
    int_z = cross(new_x, int_y)     #intermediate step 2; check notes for explaination
    int_y = cross(int_z, new_x)     #intermediate step 3; check notes for explaination
    new_y = cos(roll)*int_y + sin(roll)*cross(new_x, int_y)     #for further info check notes
    new_z = cross(new_x, new_y)

    #Setting arrow vectors and rocket object to new frame of reference
    new_xarrow.axis = new_x
    new_zarrow.axis = new_z
    new_yarrow.axis = new_y
    rocket.axis = new_x
    rocket.up = new_y

    #Ensuring the axis has the same magnitude after changes done during vector arithmetic
    new_zarrow.length = 2.25
    new_xarrow.length = 2.25
    new_yarrow.length = 2.25

    """This is a method to visualise the influence of changing pitch and yaw
        for yaw in np.arange(0, 2*np.pi, 0.01):
            rate(50)
            pitch = 10*DEG_TO_RAD
            roll = -5*DEG_TO_RAD
    """