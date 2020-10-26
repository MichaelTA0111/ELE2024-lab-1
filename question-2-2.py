import numpy as np
import matplotlib.pyplot as plt
from Car import Car
from PdController import PdController as PdCtrl


# Declare the global variables
sampling_rate = 40  # Frequency in Hz
t_final = 50  # The final time of the simulation
t_sampling = 1. / sampling_rate  # Time between consecutive samples
ticks = sampling_rate * t_final  # Total number of samples taken

# Declare the arrays to store all of the car objects, x caches, y caches, and PD controllers
car = []
x_cache = []
y_cache = []
pd_controller = [PdCtrl(0.1, 0.02, t_sampling),
                 PdCtrl(0.1, 0.1, t_sampling),
                 PdCtrl(0.1, 0.2, t_sampling),
                 PdCtrl(0.1, 0.5, t_sampling)]

# Simulation of the car with kp = 0.1 and 4 different values of kd
for i in range(4):
    car.append(Car(y_position=0.3, offset_bias=np.deg2rad(1)))  # Create a new car to be appended to the car array
    x_cache.append(np.array([car[i].get_x()]))  # Create a new x cache to be appended to the array of x caches
    y_cache.append(np.array([car[i].get_y()]))  # Create a new y cache to be appended to the array of y caches
    for t in range(ticks):
        car_steering_angle = pd_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling, 0)
        x_cache[i] = np.vstack((x_cache[i], [car[i].get_x()]))
        y_cache[i] = np.vstack((y_cache[i], [car[i].get_y()]))

# Plot all of the x-y trajectories of the car simulations on one graph
plt.plot(x_cache[0], y_cache[0], label="K$_d$ = 0.02")
plt.plot(x_cache[1], y_cache[1], label="K$_d$ = 0.1")
plt.plot(x_cache[2], y_cache[2], label="K$_d$ = 0.2")
plt.plot(x_cache[3], y_cache[3], label="K$_d$ = 0.5")
plt.grid()
plt.xlabel('x Position (m)')
plt.ylabel('y Position (m)')
plt.legend()
plt.savefig('figures\\question_2_2.svg', format='svg')  # Save the graph as a .svg file
plt.show()
