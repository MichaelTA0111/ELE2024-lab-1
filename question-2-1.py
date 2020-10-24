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


class PController:
    """
    Class to define the PController object
    """

    def __init__(self,
                 kp,
                 ts):
        """
        Constructor for the PidController class
        :param kp: The continuous-time gain for the proportional controller
        :param ts: The sampling time of the controller
        """
        self.__kp = kp
        self.__ts = ts

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

        return u


sampling_rate = 40
t_final = 50
t_sampling = 1. / sampling_rate
ticks = sampling_rate * t_final

car_0 = Car(y_position=0.3, offset_bias=np.deg2rad(1))
p_0 = PController(0.02, t_sampling)
x_cache_0 = np.array([car_0.get_x()])
y_cache_0 = np.array([car_0.get_y()])
for t in range(ticks):
    car_steering_angle = p_0.control(car_0.get_y())
    car_0.move(car_steering_angle, t_sampling)
    x_cache_0 = np.vstack((x_cache_0, [car_0.get_x()]))
    y_cache_0 = np.vstack((y_cache_0, [car_0.get_y()]))

car_1 = Car(y_position=0.3, offset_bias=np.deg2rad(1))
p_1 = PController(0.1, t_sampling)
x_cache_1 = np.array([car_1.get_x()])
y_cache_1 = np.array([car_1.get_y()])
for t in range(ticks):
    car_steering_angle = p_1.control(car_1.get_y())
    car_1.move(car_steering_angle, t_sampling)
    x_cache_1 = np.vstack((x_cache_1, [car_1.get_x()]))
    y_cache_1 = np.vstack((y_cache_1, [car_1.get_y()]))

car_2 = Car(y_position=0.3, offset_bias=np.deg2rad(1))
p_2 = PController(0.2, t_sampling)
x_cache_2 = np.array([car_2.get_x()])
y_cache_2 = np.array([car_2.get_y()])
for t in range(ticks):
    car_steering_angle = p_2.control(car_2.get_y())
    car_2.move(car_steering_angle, t_sampling)
    x_cache_2 = np.vstack((x_cache_2, [car_2.get_x()]))
    y_cache_2 = np.vstack((y_cache_2, [car_2.get_y()]))

car_3 = Car(y_position=0.3, offset_bias=np.deg2rad(1))
p_3 = PController(0.5, t_sampling)
x_cache_3 = np.array([car_3.get_x()])
y_cache_3 = np.array([car_3.get_y()])
for t in range(ticks):
    car_steering_angle = p_3.control(car_3.get_y())
    car_3.move(car_steering_angle, t_sampling)
    x_cache_3 = np.vstack((x_cache_3, [car_3.get_x()]))
    y_cache_3 = np.vstack((y_cache_3, [car_3.get_y()]))

plt.plot(x_cache_0, y_cache_0, label="K$_p$ = 0.02")
plt.plot(x_cache_1, y_cache_1, label="K$_p$ = 0.1")
plt.plot(x_cache_2, y_cache_2, label="K$_p$ = 0.2")
plt.plot(x_cache_3, y_cache_3, label="K$_p$ = 0.5")
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('y (m)')
plt.legend()
plt.show()
