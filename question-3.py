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
        :param length: The distance between the car axles in metres
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


# Declare the 2 dimensional array of Car objects
car = [[Car(velocity=1), Car(velocity=2), Car(velocity=5), Car(velocity=10)],
       [Car(length=1), Car(length=2), Car(length=5), Car(length=10)],
       [Car(), Car(pose=np.deg2rad(90)), Car(pose=np.deg2rad(180)), Car(pose=np.deg2rad(270))]]

# Declare the global variables
dt = 500
car_steering_angle = np.deg2rad(-5)
num_points = 1000  # The resolution of the graph
car_trajectory = []

# Simulation of the car with different initial values of v, L, and theta
for j in range(3):
    # Add an empty array within the car trajectory array to aid plotting the graphs later
    car_trajectory.append([])

    for i in range(4):
        # Append the solution of the system dynamics to the car trajectory array
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
    if j == 0:
        plt.savefig('figures\\question_3_a.svg', format='svg')
    if j == 1:
        plt.savefig('figures\\question_3_b.svg', format='svg')
    if j == 2:
        plt.savefig('figures\\question_3_c.svg', format='svg')
    plt.show()
