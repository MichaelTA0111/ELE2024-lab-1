import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class Car:
    """
    Class to define the Car object
    """

    def __init__(self, length=2.3,
                 velocity=5.,
                 x_position=0.,
                 y_position=0.,
                 offset_bias=0.,
                 pose=0.):
        """
        Constructor for the Car class
        :param length: The length of the car in metres
        :param velocity: The velocity of the car in metres per second
        :param x_position: The x position of the car in metres
        :param y_position: The y position of the car in metres
        :param pose: The angle the car makes with the positive x axis in an anticlockwise direction
        """
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__offset_bias = offset_bias
        self.__pose = pose

    def move(self, steering_angle, dt):
        """
        Function to make the Car object move according to the dynamics of the system
        :param steering_angle: The steering angle of the car in radians
        :param dt: The difference between the end and start times in seconds
        :return: The solution describing the system dynamics over time
        """

        # Step 1 - Define the system dynamics
        def system_dynamics(t, z):
            """
            Defines the system dynamics given in equations 3.11a-c
                        [ v * cos(theta) ]
            g(t, z)   = [ v * sin(theta) ]
                        [ v * tan(u + w) / L ]
            :param t: The variable time, which is unused in this system of equations
            :param z: An array containing the x position, y position and pose of the system
            :return: The calculated results for each equation inside function g(t, z)
            """
            theta = z[2]
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle + self.__offset_bias) / self.__length]

        # Step 2 - Define the initial condition z(0) = [x(0), y(0), theta(0)]
        z_initial = [self.__x_position,
                     self.__y_position,
                     self.__pose]

        # Step 3 - Call solve_ivp
        solution = solve_ivp(system_dynamics,
                             [0, dt],
                             z_initial)

        # Step 4 - Store the final values of x, y, and theta in the Car object
        self.__x_position = solution.y[0][-1]
        self.__y_position = solution.y[1][-1]
        self.__pose = solution.y[2][-1]

    def get_x(self):
        """
        Getter for the x position of the Car object
        :return: The x position of the Car object
        """
        return self.__x_position

    def get_y(self):
        """
        Getter for the y position of the Car object
        :return: The y position of the Car object
        """
        return self.__y_position

    def get_pose(self):
        """
        Getter for the angle the car makes with the positive x axis in an anticlockwise direction
        :return: The angle the car makes with the positive x axis in an anticlockwise direction
        """
        return self.__pose


class PidController:
    """
    Class to define the PController object
    """

    def __init__(self,
                 kp,
                 kd,
                 ki,
                 ts):
        """
        Constructor for the PidController class
        :param kp: The continuous-time gain for the proportional controller
        :param kd: The continuous-time gain for the differential controller
        :param ki: The continuous-time gain for the integral controller
        :param ts: The sampling time of the controller
        """
        self.__kp = kp
        self.__kd = kd / ts  # Discrete-time kd
        self.__ki = ki * ts  # Discrete-time ki
        self.__ts = ts
        self.__error_previous = None  # The error recorded the previous time it was calculated
        self.__sum_errors = 0.  # The sum of all previous errors calculated
        self.__u = 0.

    def control(self, y, set_point=0.):
        """
        Method to calculate the control error
        :param y: The measured value of y
        :param set_point: The set point value of y
        :return: The PID control variable
        """
        # Calculate the error
        error = set_point - y

        # Define u from the proportional controller
        u = self.__kp * error

        # Add to u based on the differential controller
        if self.__error_previous is not None:
            u += self.__kd * (error - self.__error_previous)

        # Add to u based on the integral controller
        u += self.__ki * self.__sum_errors

        self.__error_previous = error  # Store the calculated error as the previous error for future use
        self.__sum_errors += error  # Add the error to the sum of all previous errors
        self.__u = u

        return u

    def get_u(self):
        """
        Getter for the steering angle
        :return: The steering angle
        """
        return self.__u


# Declare the global variables
sampling_rate = 40
t_final = 50
t_sampling = 1. / sampling_rate
ticks = sampling_rate * t_final

# Declare the arrays to store all of the car objects, x caches, y caches, u caches, and P controllers
car = []
x_cache = []
y_cache = []
u_cache = []
pid_controller = [PidController(0.5, 0.3, 0.02, t_sampling),
                  PidController(0.5, 0.3, 0.1, t_sampling),
                  PidController(0.5, 0.3, 0.2, t_sampling),
                  PidController(0.5, 0.3, 0.5, t_sampling)]

# Simulation of the car with kp = 0.1 and 4 different values of kd
for i in range(4):
    car.append(Car(y_position=0.3, offset_bias=np.deg2rad(1)))  # Create a new car to be appended to the car array
    x_cache.append(np.array([car[i].get_x()]))  # Create a new x cache to be appended to the array of x caches
    y_cache.append(np.array([car[i].get_y()]))  # Create a new y cache to be appended to the array of y caches
    # Create a new u cache to be appended to the array of u caches
    u_cache.append(np.array([pid_controller[i].get_u()]))
    for t in range(ticks):
        car_steering_angle = pid_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling)
        x_cache[i] = np.vstack((x_cache[i], [car[i].get_x()]))
        y_cache[i] = np.vstack((y_cache[i], [car[i].get_y()]))
        u_cache[i] = np.vstack((u_cache[i], [pid_controller[i].get_u()]))

# Plot all of the u trajectories of the car simulations on one graph
t_span = t_sampling * np.arange(ticks + 1)
plt.plot(t_span, u_cache[0], label="K$_i$ = 0.02")
plt.plot(t_span, u_cache[1], label="K$_i$ = 0.1")
plt.plot(t_span, u_cache[2], label="K$_i$ = 0.2")
plt.plot(t_span, u_cache[3], label="K$_i$ = 0.5")
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('Steering Angle (rad)')
plt.legend()
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
plt.show()
