import math
import random
import numpy as np
import matplotlib.pyplot as plt

DEG_TO_RAD  = 0.01745329251

class Simulation:
    def __init__(self, run_time, start_pitch, edf_thrust, distance_com, moment_of_inertia, mass, cross_sectional_area):
        self.transient = run_time
        self.initial = start_pitch
        self.thrust = edf_thrust
        self.com = distance_com
        self.moi = moment_of_inertia
        self.mass = mass
        self.csa = cross_sectional_area
        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

        self.time = [0]
        self.pitch = []
        self.eff_thrust = []
        self.pid_output = []
        self.ang_vel = [0]
        self.ang_acc = [0]
        self.cross_wind = [3, 3, 3]
        self.control_torque = []
        self.wind_torque = []
        self.total_torque = []

    def insert_gain(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def control_thrust(self, i):
        pid = math.radians(self.pid_output[i])
        horizontal_vector = self.thrust*math.sin(pid)
        self.eff_thrust.append(horizontal_vector)

    def wind(self, i):
        wind_bias = 3.0
        wind_variation = random.uniform(-1, 1)
        weighted_sum = self.cross_wind[i] + self.cross_wind[i+1] + self.cross_wind[i+2] + wind_variation + 2*wind_bias + 2
        weighted_average = weighted_sum/6
        weighted_average = self.bound_check(wind_bias, weighted_average)
        self.cross_wind.append(weighted_average)
        if(weighted_average < 0):
            crosswind_force = -1.229*(self.cross_wind[i+3]**2)
        else:
            crosswind_force = 1.229*(self.cross_wind[i+3]**2)
        return crosswind_force
    
    def drag(self, i):
        coefficient = self.thrust/self.mass + 9.81
        rocket_velocity = coefficient*self.time[i]
        drag = 0.5*1.229*(rocket_velocity**2)*0.75*self.csa
        return drag

    def torque(self, i):
        self.control_thrust(i)
        control_torque = self.eff_thrust[i]*self.com
        self.control_torque.append(control_torque)

        crosswind_force = self.wind(i)
        drag = self.drag(i)
        if drag == 0.0:
            ang_attack = 0.0
        else:
            ang_attack = np.arctan(crosswind_force / drag)
        wind_force = math.sqrt((crosswind_force)**2 + drag**2)
        nose_torque = 0.3*(wind_force*math.sin(ang_attack))*self.com
        tail_torque = 0.6*(wind_force*math.sin(ang_attack))*self.com
        wind_torque = tail_torque - nose_torque
        self.wind_torque.append(wind_torque)
        total = control_torque #+ wind_torque
        self.total_torque.append(total)

        return total

    def tvc_physics(self, Kp, Ki, Kd):
        dt = 0.01
        self.pitch.append(self.initial)
        self.pid_output.append(self.initial)
        iteration = int(self.transient/dt)
        for i in range(iteration):
            torque = self.torque(i)
            ang_acc = (torque)/self.moi
            self.ang_acc.append(math.degrees(ang_acc))
            integral = self.ang_vel[i] + self.ang_acc[i+1]*dt
            self.ang_vel.append(integral)
            incremental_change = -self.ang_vel[i+1]*dt
            self.pitch.append(self.pitch[i] + incremental_change)

            output = Kp*self.pitch[i+1] + Kd*(self.pitch[i+1] - self.pitch[i])/dt + Ki*self.pitch[i+1]*dt
            output = self.bound_check(0.122, output)
            self.pid_output.append(output)
            self.time.append(self.time[i] + dt)

    def bound_check(self, bound, value):
        if abs(value) > bound:
            if value < 0.0:
                return -bound
            else:
                return bound
        else:
            return value
    
    def get_graph(self):
        self.tvc_physics(self.Kp, self.Ki, self.Kd)
        
        fig = plt.figure()

        ax1 = fig.add_subplot(2, 2, 1)
        ax1.plot(self.time, self.cross_wind[0:len(self.cross_wind) - 2])
        ax1.set_ylabel("Cross-wind [ms^-1]")
        ax1.set_xlabel("Time [s]")

        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(self.time, self.pitch) 
        ax2.set_ylabel("Pitch [rad]")
        ax2.set_xlabel("Time [s]")

        ax3 = fig.add_subplot(2, 2, 3)
        ax3.plot(self.time[0:len(self.time) - 1], self.wind_torque)
        ax3.set_ylabel("Wind Torque [Nm]")
        ax3.set_xlabel("Time [s]") 

        ax4 = fig.add_subplot(2, 2, 4)
        ax4.plot(self.time[0:len(self.time) - 1], self.control_torque)
        ax4.set_ylabel("Control Torque [Nm]")
        ax4.set_xlabel("Time [s]") 

        plt.show()

run_time = 3
start_pitch = 7*DEG_TO_RAD
edf_thrust = 100
distance_com = 0.102
moment_of_inertia = 0.035
mass = 2.82
cross_sectional_area = 0.045

tvc = Simulation(run_time, start_pitch, edf_thrust, distance_com, moment_of_inertia, mass, cross_sectional_area)

#Kp = 0.255
#Ki = 180
#Kd = 0.009
Kp = 0.7
Ki = 0.01
Kd = 0.1
tvc.insert_gain(Kp, Ki, Kd)
tvc.get_graph()