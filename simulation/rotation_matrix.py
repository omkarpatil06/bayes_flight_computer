from vpython import *       #imports everything from python
import numpy as np

scene = canvas(range = 5, width = 800, height = 800, background = color.black, forward = vector(-0.15, -0.25, -0.25))

#New frame of reference
def rotation(phi, theta, psi):
    phi = np.radians(phi)
    theta = np.radians(theta)
    psi = np.radians(psi)
    r_x = np.array([[1, 0, 0], [0, np.cos(phi), -1*np.sin(phi)], [0, np.sin(phi), np.cos(phi)]])
    r_y = np.array([[np.cos(theta), 0, np.sin(theta)], [0, 1, 0], [-1*sin(theta), 0, np.cos(theta)]])
    r_z = np.array([[np.cos(psi), -1*np.sin(psi), 0], [np.sin(psi), np.cos(psi), 0], [0, 0, 1]])
    return np.matmul(np.matmul(r_z, r_y), r_x)

# angles of imu
phi = 30   #roll
theta = 20  #pitch
psi = 0    #yaw

# angle difference between inital and servo axis
sphi = 0   #roll
stheta = 0  #pitch
spsi = 30    #yaw

#imu basis vectors
axis = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
xarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.red, pos = vector(0, 0, 0), axis=vector(axis[0][0], axis[0][1], axis[0][2]))             #arrow in x-direction
yarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.green, pos = vector(0, 0, 0), axis=vector(axis[1][0], axis[1][1], axis[1][2]))           #arrow in y-direction
zarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.blue, pos = vector(0, 0, 0), axis=vector(axis[2][0], axis[2][1], axis[2][2]))            #arrow in z-direction

#imu changed basis vectors
new_axis = np.matmul(axis, rotation(phi, theta, psi))
new_xarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.red, pos = vector(0, 0, 0), axis=vector(new_axis[0][0], new_axis[0][1], new_axis[0][2]))             #arrow in x-direction
new_yarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.green, pos = vector(0, 0, 0), axis=vector(new_axis[1][0], new_axis[1][1], new_axis[1][2]))           #arrow in y-direction
new_zarrow = arrow(length = 2.25, shaftwidth = 0.1, color=color.blue, pos = vector(0, 0, 0), axis=vector(new_axis[2][0], new_axis[2][1], new_axis[2][2]))            #arrow in z-direction

#initial servo basis vectors
saxis= np.matmul(axis, rotation(sphi, stheta, spsi))
s_xarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.cyan, pos = vector(0, 0, 0), axis=vector(saxis[0][0], saxis[0][1], saxis[0][2]))             #arrow in x-direction
s_yarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.magenta, pos = vector(0, 0, 0),axis=vector(saxis[1][0], saxis[1][1], saxis[1][2]))           #arrow in y-direction
s_zarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.yellow, pos = vector(0, 0, 0), axis=vector(saxis[2][0], saxis[2][1], saxis[2][2]))            #arrow in z-direction

#changed servo basis vectors
new_saxis= np.matmul(new_axis, rotation(sphi, stheta, spsi))
new_sxarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.cyan, pos = vector(0, 0, 0), axis=vector(new_saxis[0][0], new_saxis[0][1], new_saxis[0][2]))             #arrow in x-direction
new_syarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.magenta, pos = vector(0, 0, 0),axis=vector(new_saxis[1][0], new_saxis[1][1], new_saxis[1][2]))           #arrow in y-direction
new_szarrow = arrow(length = 1.5, shaftwidth = 0.1, color=color.yellow, pos = vector(0, 0, 0), axis=vector(new_saxis[2][0], new_saxis[2][1], new_saxis[2][2]))            #arrow in z-direction

