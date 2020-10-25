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
        :param length: The distance between the car axles in metres
        :param velocity: The velocity of the car in metres per second
        :param x_position: The x position of the car in metres
        :param y_position: The y position of the car in metres
        :param offset_bias: The constant bias of the actuator in radians
        :param pose: The angle the car makes with the positive x axis in an anticlockwise direction in radians
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


# Declare the global variables
sampling_rate = [10, 20, 40, 100]
t_final = 50
t_sampling = [1. / sampling_rate[0],
              1. / sampling_rate[1],
              1. / sampling_rate[2],
              1. / sampling_rate[3]]
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
    pid_controller.append(PidController(0.5, 0.05, 0.01, t_sampling[i]))
    for t in range(ticks[i]):
        car_steering_angle = pid_controller[i].control(car[i].get_y())
        car[i].move(car_steering_angle, t_sampling[i])
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
plt.savefig('figures\\question_2_4.svg', format='svg')
plt.show()
