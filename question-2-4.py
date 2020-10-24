# import numpy as np
# from scipy.integrate import solve_ivp
# import matplotlib.pyplot as plt
#
#
# class Car:
#     """
#     Class to define the Car object
#     """
#
#     def __init__(self, length=2.3,
#                  velocity=5.,
#                  x_position=0.,
#                  y_position=0.,
#                  offset_bias=0.,
#                  pose=0.):
#         """
#         Constructor for the Car class
#         :param length: The length of the car in metres
#         :param velocity: The velocity of the car in metres per second
#         :param x_position: The x position of the car in metres
#         :param y_position: The y position of the car in metres
#         :param pose: The angle the car makes with the positive x axis in an anticlockwise direction
#         """
#         self.__length = length
#         self.__velocity = velocity
#         self.__x_position = x_position
#         self.__y_position = y_position
#         self.__offset_bias = offset_bias
#         self.__pose = pose
#
#     def move(self, steering_angle, dt=0.001):
#         """
#         Function to make the Car object move according to the dynamics of the system
#         :param steering_angle: The steering angle of the car in radians
#         :param dt: The difference between consecutive time values in seconds
#         :return: The solution describing the system dynamics over time
#         """
#
#         # Step 1 - Define the system dynamics
#         def system_dynamics(t, z):
#             """
#             Defines the system dynamics given in equations 3.11a-c
#                         [ v * cos(theta) ]
#             g(t, z)   = [ v * sin(theta) ]
#                         [ v * tan(u + w) / L ]
#             :param t: The variable time, which is unused in this system of equations
#             :param z: An array containing the x position, y position and pose of the system
#             :return: The calculated results for each equation inside function g(t, z)
#             """
#             theta = z[2]
#             return [self.__velocity * np.cos(theta),
#                     self.__velocity * np.sin(theta),
#                     self.__velocity * np.tan(steering_angle + self.__offset_bias) / self.__length]
#
#         # Step 2 - Define the initial condition z(0) = [x(0), y(0), theta(0)]
#         z_initial = [self.__x_position,
#                      self.__y_position,
#                      self.__pose]
#
#         # Step 3 - Call solve_ivp
#         solution = solve_ivp(system_dynamics,
#                              [0., t_final],
#                              z_initial)
#
#         # Step 4 - Store the final values of x, y, and theta in the Car object
#         self.__x_position = solution.y[0][-1]
#         self.__y_position = solution.y[1][-1]
#         self.__pose = solution.y[2][-1]
#
#         # Step 5 - Return the solution describing the system dynamics over time
#         return solution
#
#     def get_x(self):
#         """
#         Getter for the x position of the Car object
#         :return: The x position of the Car object
#         """
#         return self.__x_position
#
#     def get_y(self):
#         """
#         Getter for the y position of the Car object
#         :return: The y position of the Car object
#         """
#         return self.__y_position
#
#     def get_pose(self):
#         """
#         Getter for the angle the car makes with the positive x axis in an anticlockwise direction
#         :return: The angle the car makes with the positive x axis in an anticlockwise direction
#         """
#         return self.__pose
#
#
# class PidController:
#     """
#     Class to define the PidController object
#     """
#
#     def __init__(self,
#                  kp,
#                  kd,
#                  ki,
#                  ts):
#         """
#         Constructor for the PidController class
#         :param kp: The continuous-time gain for the proportional controller
#         :param kd: The continuous-time gain for the differential controller
#         :param ki: The continuous-time gain for the integral controller
#         :param ts: The sampling time of the controller
#         """
#         self.__kp = kp
#         self.__kd = kd / ts     # Discrete-time kd
#         self.__ki = ki * ts     # Discrete-time ki
#         self.__ts = ts
#         self.__error_previous = None    # The error recorded the previous time it was calculated
#         self.__sum_errors = 0           # The sum of all previous errors calculated
#
#     def control(self, y, set_point=0.):
#         """
#         Method to calculate the control error
#         :param y: The measured value of y
#         :param set_point: The set point value of y
#         :return: The PID control variable
#         """
#         # Calculate the error
#         error = set_point - y
#
#         # Define u from the proportional controller
#         u = self.__kp * error
#
#         # Add to u based on the differential controller
#         if self.__error_previous is not None:
#             u += self.__kd * (error - self.__error_previous)
#
#         # Add to u based on the integral controller
#         u += self.__ki * self.__sum_errors
#
#         self.__error_previous = error   # Store the calculated error as the previous error for future use
#         self.__sum_errors += error      # Add the error to the sum of all previous errors
#
#         return u
#
#
# car_2 = Car(y_position=0.3, offset_bias=np.deg2rad(1))
# num_points = 1000   # The resolution of the graph
# t_sampling = 0.025  # Sampling frequency of 40 Hz
# t_final = 50.
# ticks = 2000        # 40 Hz * 50 s = 2000
# pid = PidController(0.25, 0., 0., t_sampling)
#
# x_cache = np.array([car_2.get_x()])
# y_cache = np.array([car_2.get_y()])
# for t in range(ticks):
#     car_steering_angle = pid.control(car_2.get_y())
#     car_2.move(car_steering_angle, t_sampling)
#     x_cache = np.vstack((x_cache, [car_2.get_x()]))
#     y_cache = np.vstack((y_cache, [car_2.get_y()]))
#
# t_span = t_sampling * np.arange(ticks + 1)
# plt.plot(t_span, y_cache)
# plt.grid()
# plt.xlabel('Time (s)')
# plt.ylabel('y (m)')
# plt.show()

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class Car:

    def __init__(self, length=2.3, velocity=5., disturbance=0, x=0., y=0., pose=0.):
        self.__length = length
        self.__velocity = velocity
        self.__disturbance = disturbance
        self.__x = x
        self.__y = y
        self.__pose = pose

    # Simulate the motion of the car from t = 0 to t = 0 + dt.
    def move(self, steering_angle, dt):
        # Define the system dynamics as a function for equations 3.11 a - c
        def bicycle_model(t, z):
            #           [v * cos(theta)]
            # g(t,z) =  [v * sin(theta)]
            #           [v * tan(u)/L]
            theta = z[2]
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle + self.__disturbance) / self.__length]

        z_initial = [self.__x, self.__y, self.__pose]  # Starting from z_initial = [self.x, self.y, self.pose]
        solution = solve_ivp(bicycle_model,
                             [0, dt],
                             z_initial)
        self.__x = solution.y[0][-1]
        self.__y = solution.y[1][-1]
        self.__pose = solution.y[2][-1]

    def x(self):
        return self.__x

    def y(self):
        return self.__y

    def theta(self):
        return self.__pose

    def length(self):
        return self.__length

    def velocity(self):
        return self.__velocity

    def disturbance(self):
        return self.__disturbance


class PidController:

    def __init__(self, kp, kd, ki, ts):
        """
        #Constructor for PID Controller
        :param kp:
        :param ki:
        :param kd:
        :param ts:
        """
        self.__kp = kp  # kp = proportional gain
        self.__kd = kd / ts  # kd = derivative gain
        self.__ki = ki * ts  # ki = integral gain
        self.__ts = ts  # ts = sampling time
        self.__previous_error = None  # 'Not defined yet'
        self.__sum_errors = 0.0
        self.control_action = 0.  # Control action (steering angle) for steering_cache

    def control(self, y, y_set_point=0):
        error = y_set_point - y  # Calculates the control error
        control_action = self.__kp * error  # P control

        if self.__previous_error is not None:
            control_action += self.__kd * (error - self.__previous_error)  # D control

        control_action += self.__ki * self.__sum_errors  # I Control

        self.__sum_errors += error
        self.__previous_error = error  # Means that next time we need the previous error
        self.control_action = control_action  # For steering_cache
        return control_action


# Initial variables, sampling rate and ticks
sampling_rate = 40  # Sampling rate in Hz
t_final = 50  # t [0, 50]
x_initial = 0
y_initial = 0.3  # 0.3 m = 30 cm
theta_initial = np.deg2rad(5)  # 5° in radians
disturbance_initial = np.deg2rad(1)  # 1° in radians
sampling_period = 1 / sampling_rate
ticks = sampling_rate * t_final  # 40 Hz x 50 s = 2000

# Simulation of vehicle with kp = 0.9, kd = 0.5 and ki = 0.01
murphy = Car(x=x_initial, y=y_initial, pose=theta_initial, disturbance=disturbance_initial)
pid_1 = PidController(kp=0.9, kd=0.5, ki=0.01, ts=sampling_period)
y_cache = np.array([murphy.y()])  # Inserted current first value of y into the cache
x_cache = np.array([murphy.x()])  # Inserted current first value of x into the cache
steering_cache = np.array([pid_1.control_action])
for k in range(ticks):
    control_action = pid_1.control(murphy.y())
    murphy.move(control_action, sampling_period)
    y_cache = np.vstack((y_cache, [murphy.y()]))
    x_cache = np.vstack((x_cache, [murphy.x()]))
    steering_cache = np.vstack((steering_cache, [pid_1.control_action]))

# Simulation of vehicle with kp = 0.9, kd = 0.5 and ki = 0.08. These are my ideal parameters.
murphy_2 = Car(x=x_initial, y=y_initial, pose=theta_initial, disturbance=disturbance_initial)
pid_2 = PidController(kp=0.9, kd=0.5, ki=0.08, ts=sampling_period)
y_cache_2 = np.array([murphy_2.y()])
x_cache_2 = np.array([murphy_2.x()])
steering_cache_2 = np.array([pid_2.control_action])
for k in range(ticks):
    control_action_2 = pid_2.control(murphy_2.y())
    murphy_2.move(control_action_2, sampling_period)
    y_cache_2 = np.vstack((y_cache_2, [murphy_2.y()]))
    x_cache_2 = np.vstack((x_cache_2, [murphy_2.x()]))
    steering_cache_2 = np.vstack((steering_cache_2, [pid_2.control_action]))

# Simulation of vehicle with kp = 0.9, kd = 0.5 and ki = 0.7
murphy_3 = Car(x=x_initial, y=y_initial, pose=theta_initial, disturbance=disturbance_initial)
pid_3 = PidController(kp=0.9, kd=0.5, ki=0.7, ts=sampling_period)
y_cache_3 = np.array([murphy_3.y()])
x_cache_3 = np.array([murphy_3.x()])
steering_cache_3 = np.array([pid_3.control_action])
for k in range(ticks):
    control_action_3 = pid_3.control(murphy_3.y())
    murphy_3.move(control_action_3, sampling_period)
    y_cache_3 = np.vstack((y_cache_3, [murphy_3.y()]))
    x_cache_3 = np.vstack((x_cache_3, [murphy_3.x()]))
    steering_cache_3 = np.vstack((steering_cache_3, [pid_3.control_action]))

# Plot the graphs of the (x, y) Trajectories
plt.plot(x_cache, y_cache, label="K$_i$ = 0.01")
plt.plot(x_cache_2, y_cache_2, label="K$_i$ = 0.08")
plt.plot(x_cache_3, y_cache_3, label="K$_i$ = 0.7")
plt.grid()
plt.xlabel('X - Trajectory (m)')
plt.ylabel('Y - Trajectory (m)')
plt.legend()
plt.show()

# Plot graphs of u(t) against time
t_span = sampling_period * np.arange(ticks + 1)
plt.plot(t_span, steering_cache, label="K$_i$ = 0.01")
plt.plot(t_span, steering_cache_2, label="K$_i$ = 0.08")
plt.plot(t_span, steering_cache_3, label="K$_i$ = 0.7")
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (rad)')
plt.legend()
plt.show()