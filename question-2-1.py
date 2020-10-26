import numpy as np
import matplotlib.pyplot as plt
from Car import Car
from PController import PController as PCtrl


# Declare the global variables
sampling_rate = 40
t_final = 50
t_sampling = 1. / sampling_rate
ticks = sampling_rate * t_final

# Declare the arrays to store all of the car objects, x caches, y caches, and P controllers
car = []
x_cache = []
y_cache = []
p_controller = [PCtrl(0.02, t_sampling),
                PCtrl(0.1, t_sampling),
                PCtrl(0.2, t_sampling),
                PCtrl(0.5, t_sampling)]

# Simulation of the car with 4 different values of kp
for i in range(4):
    car.append(Car(y_position=0.3, offset_bias=np.deg2rad(1)))  # Create a new car to be appended to the car array
    x_cache.append(np.array([car[i].get_x()]))  # Create a new x cache to be appended to the array of x caches
    y_cache.append(np.array([car[i].get_y()]))  # Create a new y cache to be appended to the array of y caches
    for t in range(ticks):
        car_steering_angle = p_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling, 0)
        x_cache[i] = np.vstack((x_cache[i], [car[i].get_x()]))
        y_cache[i] = np.vstack((y_cache[i], [car[i].get_y()]))

# Plot all of the x-y trajectories of the car simulations on one graph
plt.plot(x_cache[0], y_cache[0], label="K$_p$ = 0.02")
plt.plot(x_cache[1], y_cache[1], label="K$_p$ = 0.1")
plt.plot(x_cache[2], y_cache[2], label="K$_p$ = 0.2")
plt.plot(x_cache[3], y_cache[3], label="K$_p$ = 0.5")
plt.grid()
plt.xlabel('x Position (m)')
plt.ylabel('y Position (m)')
plt.legend()
plt.savefig('figures\\question_2_1.svg', format='svg')
plt.show()
