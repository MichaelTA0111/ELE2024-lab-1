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
                 pose=0.):
        """
        Constructor for the Car class
        :param length: The length of the car in metres
        :param velocity: The velocity of the car in metres per second
        :param x_position: The x position of the car in metres
        :param y_position: The y position of the car in metres
        :param pose: The angle the car makes with the positive x axis in an anticlockwise direction in radians
        """
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
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
            Defines the system dynamics given in equations 4.1a-c
                        [ v * cos(theta) ]
            g(t, z)   = [ v * sin(theta) ]
                        [ v * tan(u) / L ]
            :param t: The variable time, which is unused in this system of equations
            :param z: An array containing the x position, y position and pose of the system
            :return: The calculated results for each equation inside function g(t, z)
            """
            theta = z[2]
            return [self.__velocity * np.cos(theta),
                    self.__velocity * np.sin(theta),
                    self.__velocity * np.tan(steering_angle) / self.__length]

        # Step 2 - Define the initial condition z(0) = [x(0), y(0), theta(0)]
        z_initial = [self.__x_position,
                     self.__y_position,
                     self.__pose]

        # Step 3 - Call solve_ivp
        solution = solve_ivp(system_dynamics,
                             [0, dt],
                             z_initial,
                             t_eval=np.linspace(0, dt, num_points))

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


# Declare the 2 dimensional array of Car objects
car = [[Car(velocity=1), Car(velocity=2), Car(velocity=5), Car(velocity=10)],
       [Car(length=1), Car(length=2), Car(length=5), Car(length=10)],
       [Car(), Car(pose=np.deg2rad(90)), Car(pose=np.deg2rad(180)), Car(pose=np.deg2rad(270))]]

# Declare the global variables
sampling_rate = 40
t_sampling = 1 / sampling_rate
dt = 500
car_steering_angle = np.deg2rad(-5)
num_points = 1000  # The resolution of the graph
pid_controller = []
car_trajectory = []

# Simulation of the car with kp = 0.5, kd = 0.05, ki = 0.01, and different initial values of v, L, and theta
for j in range(3):
    # Add an empty array within the pid controller and car trajectory arrays to aid plotting the graphs later
    pid_controller.append([])
    car_trajectory.append([])
    for i in range(4):
        # Create a new PID controller to be appended to the corresponding array of PID controllers
        pid_controller[j].append(PidController(0.5, 0.05, 0.01, t_sampling))
        car_trajectory[j].append(car[j][i].move(car_steering_angle, dt))

    if j == 0:
        # Plot all of the x-y trajectories of the cars simulations on one graph, varying initial velocity
        plt.plot(car_trajectory[j][0].y[0], car_trajectory[j][0].y[1].T, label="v = 1 m/s")
        plt.plot(car_trajectory[j][1].y[0], car_trajectory[j][1].y[1].T, label="v = 2 m/s")
        plt.plot(car_trajectory[j][2].y[0], car_trajectory[j][2].y[1].T, label="v = 5 m/s")
        plt.plot(car_trajectory[j][3].y[0], car_trajectory[j][3].y[1].T, label="v = 10 m/s")

    if j == 1:
        # Plot all of the x-y trajectories of the cars simulations on one graph, varying car length
        plt.plot(car_trajectory[j][0].y[0], car_trajectory[j][0].y[1].T, label="L = 1 m")
        plt.plot(car_trajectory[j][1].y[0], car_trajectory[j][1].y[1].T, label="L = 2 m")
        plt.plot(car_trajectory[j][2].y[0], car_trajectory[j][2].y[1].T, label="L = 5 m")
        plt.plot(car_trajectory[j][3].y[0], car_trajectory[j][3].y[1].T, label="L = 10 m")

    if j == 2:
        # Plot all of the x-y trajectories of the cars simulations on one graph, varying initial pose
        plt.plot(car_trajectory[j][0].y[0], car_trajectory[j][0].y[1].T, label="Theta = 0 degrees")
        plt.plot(car_trajectory[j][1].y[0], car_trajectory[j][1].y[1].T, label="Theta = 90 degrees")
        plt.plot(car_trajectory[j][2].y[0], car_trajectory[j][2].y[1].T, label="Theta = 180 degrees")
        plt.plot(car_trajectory[j][3].y[0], car_trajectory[j][3].y[1].T, label="Theta = 270 degrees")

    plt.grid()
    plt.xlabel('x Position (m)')
    plt.ylabel('y Position (m)')
    plt.legend()
    plt.show()
