import numpy as np
import matplotlib.pyplot as plt
from Car import Car
from PidController import PidController as PidCtrl


# Declare the global variables
sampling_rate = 40  # Frequency in Hz
t_final = 50  # The final time of the simulation
t_sampling = 1. / sampling_rate  # Time between consecutive samples
ticks = sampling_rate * t_final  # Total number of samples taken

# Declare the arrays to store all of the car objects, x caches, y caches, u caches, and PID controllers
car = []
x_cache = []
y_cache = []
u_cache = []
pid_controller = [PidCtrl(0.6, 0.4, 0.01, t_sampling),
                  PidCtrl(0.6, 0.4, 0.05, t_sampling),
                  PidCtrl(0.6, 0.4, 0.1, t_sampling),
                  PidCtrl(0.6, 0.4, 0.25, t_sampling)]

# Simulation of the car with kp = 0.6, kd = 0.4, and 4 different values of kd
for i in range(4):
    car.append(Car(y_position=0.3, offset_bias=np.deg2rad(1)))  # Create a new car to be appended to the car array
    x_cache.append(np.array([car[i].get_x()]))  # Create a new x cache to be appended to the array of x caches
    y_cache.append(np.array([car[i].get_y()]))  # Create a new y cache to be appended to the array of y caches
    # Create a new u cache to be appended to the array of u caches
    u_cache.append(np.array([pid_controller[i].get_u()]))
    for t in range(ticks):
        car_steering_angle = pid_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling, 0)
        x_cache[i] = np.vstack((x_cache[i], [car[i].get_x()]))
        y_cache[i] = np.vstack((y_cache[i], [car[i].get_y()]))
        u_cache[i] = np.vstack((u_cache[i], [pid_controller[i].get_u()]))

# Plot all of the u trajectories of the car simulations on one graph
t_span = t_sampling * np.arange(ticks + 1)  # All values of time which were used for sampling
plt.plot(t_span, u_cache[0], label="K$_i$ = 0.01")
plt.plot(t_span, u_cache[1], label="K$_i$ = 0.05")
plt.plot(t_span, u_cache[2], label="K$_i$ = 0.1")
plt.plot(t_span, u_cache[3], label="K$_i$ = 0.25")
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (rad)')
plt.legend()
plt.savefig('figures\\question_2_3_a.svg', format='svg')  # Save the graph as a .svg file
plt.show()

# Plot all of the x-y trajectories of the car simulations on one graph
plt.plot(x_cache[0], y_cache[0], label="K$_i$ = 0.02")
plt.plot(x_cache[1], y_cache[1], label="K$_i$ = 0.1")
plt.plot(x_cache[2], y_cache[2], label="K$_i$ = 0.2")
plt.plot(x_cache[3], y_cache[3], label="K$_i$ = 0.5")
plt.grid()
plt.xlabel('x Position (m)')
plt.ylabel('y Position (m)')
plt.legend()
plt.savefig('figures\\question_2_3_b.svg', format='svg')  # Save the graph as a .svg file
plt.show()
