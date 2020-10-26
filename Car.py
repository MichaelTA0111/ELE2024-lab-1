import numpy as np
from scipy.integrate import solve_ivp


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
        :param pose: The angle the car makes with the positive x axis in the anticlockwise direction in radians
        """
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__offset_bias = offset_bias
        self.__pose = pose

    def move(self, steering_angle, dt, num_points):
        """
        Function to make the Car object move according to the dynamics of the system
        :param steering_angle: The steering angle of the car in radians
        :param dt: The difference between the end and start times in seconds
        :param num_points: The resolution of the graph
        :return: The solution describing the system dynamics over time
        """
        # Step 1 - Define the system dynamics
        def system_dynamics(t, z):
            """
            Defines the system dynamics given in equations 3.11a-c
            When w = 0, the system dynamics given in equations 2.1a-c and 4.1a-c are also defined
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
        if num_points != 0:
            solution = solve_ivp(system_dynamics,
                                 [0, dt],
                                 z_initial,
                                 t_eval=np.linspace(0, dt, num_points))
        else:
            solution = solve_ivp(system_dynamics,
                                 [0, dt],
                                 z_initial)

        # Step 4 - Store the final values of x, y, and theta in the Car object
        self.__x_position = solution.y[0][-1]
        self.__y_position = solution.y[1][-1]
        self.__pose = solution.y[2][-1]

        # Step 5 - Return the solution describing the system dynamics over time
        return solution

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
