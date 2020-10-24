import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class Car:

    def __init__(self, length=2.3,
                 velocity=5.,
                 x_position=0.,
                 y_position=0.,
                 pose=0.):
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__pose = pose

    def move(self, steering_angle, dt=0):
        # Step 1 - Define the system dynamics
        def system_dynamics(t, z):
            """
            Defines the system dynamics given in equations 2.1a-c
                         [ v * cos(theta) ]
             g(t, z)   = [ v * sin(theta) ]
                         [ v * tan(u) / L ]
            :param t: The variable time, which remains unused in this system of equations
            :param z: An array containing the x position, y position and pose of the system
            :return:
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
                             [0, t_final],
                             z_initial,
                             t_eval=np.linspace(0, t_final, num_points))

        self.__x_position = solution.y[0][-1]
        self.__y_position = solution.y[1][-1]
        self.__pose = solution.y[2][-1]

        return solution


car_1_1 = Car(x_position=0, y_position=0.3, pose=-np.pi / 36)
car_steering_angle = -np.pi / 90
t_final = 2
num_points = 1000
car_trajectory = car_1_1.move(car_steering_angle)

# Question 1.1 - x position (m) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[0].T)
plt.xlabel('Time (s)')
plt.ylabel('x position (m)')
plt.grid()
plt.show()

# Question 1.1 - y position (m) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[1].T)
plt.xlabel('Time (s)')
plt.ylabel('y position (m)')
plt.grid()
plt.show()

# Question 1.1 - theta (rad) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[2].T)
plt.xlabel('Time (s)')
plt.ylabel('theta (rad)')
plt.grid()
plt.show()

# Question 1.2 - y position (m) against x position (m)
plt.plot(car_trajectory.y[0], car_trajectory.y[1].T)
plt.xlabel('x position (m)')
plt.ylabel('y position (m)')
plt.grid()
plt.show()
