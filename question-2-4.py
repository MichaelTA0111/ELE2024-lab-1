import numpy as np
import matplotlib.pyplot as plt
from Car import Car
from PidController import PidController as PidCtrl


# Declare the global variables
sampling_rate = [10, 20, 40, 100]  # Frequency in Hz
t_final = 50  # The final time of the simulation
# Time between consecutive samples
t_sampling = [1. / sampling_rate[0],
              1. / sampling_rate[1],
              1. / sampling_rate[2],
              1. / sampling_rate[3]]
# Total number of samples taken
ticks = [sampling_rate[0] * t_final,
         sampling_rate[1] * t_final,
         sampling_rate[2] * t_final,
         sampling_rate[3] * t_final]

# Declare the arrays to store all of the car objects, x caches, y caches, and PID controllers
car = []
x_cache = []
y_cache = []
pid_controller = []

# Simulation of the car with kp = 0.5, kd = 0.05, ki = 0.01, and 4 different values of ts
for i in range(4):
    car.append(Car(y_position=0.3, offset_bias=np.deg2rad(1)))  # Create a new car to be appended to the car array
    x_cache.append(np.array([car[i].get_x()]))  # Create a new x cache to be appended to the array of x caches
    y_cache.append(np.array([car[i].get_y()]))  # Create a new y cache to be appended to the array of y caches
    # Create a new PidController to append to the array of PidControllers
    pid_controller.append(PidCtrl(0.5, 0.05, 0.01, t_sampling[i]))
    for t in range(ticks[i]):
        car_steering_angle = pid_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling[i], 0)
        x_cache[i] = np.vstack((x_cache[i], [car[i].get_x()]))
        y_cache[i] = np.vstack((y_cache[i], [car[i].get_y()]))

# Plot all of the x-y trajectories of the car simulations on one graph
plt.plot(x_cache[0], y_cache[0], label="T$_s$ = 0.1 s")
plt.plot(x_cache[1], y_cache[1], label="T$_s$ = 0.05 s")
plt.plot(x_cache[2], y_cache[2], label="T$_s$ = 0.025 s")
plt.plot(x_cache[3], y_cache[3], label="T$_s$ = 0.01 s")
plt.grid()
plt.xlabel('x Position (m)')
plt.ylabel('y Position (m)')
plt.legend()
plt.savefig('figures\\question_2_4.svg', format='svg')  # Save the graph as a .svg file
plt.show()
