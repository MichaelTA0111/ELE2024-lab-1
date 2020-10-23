import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


class Car:

    def __init__(self, length=2.,
                 velocity=5.,
                 x_position=0.,
                 y_position=0.,
                 pose=0.):
        self.__length = length
        self.__velocity = velocity
        self.__x_position = x_position
        self.__y_position = y_position
        self.__pose = pose

    # Step 1 - Define the system dynamics
    def move(self, steering_angle, dt):
        def system_dynamics(t, z):
            # Defines the system dynamics given in equations 2.1a-c
            #             [ v * cos(theta) ]
            # g(t, z)   = [ v * sin(theta) ]
            #             [ v * tan(u) / L ]
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
                             z_initial)

        self.__x_position = solution.y[0][-1]
        self.__y_position = solution.y[1][-1]
        self.__pose = solution.y[2][-1]
        # print(solution)

    def get_x(self):
        return self.__x_position

    def get_y(self):
        return self.__y_position

    def get_pose(self):
        return self.__pose


class PidController:

    def __init__(self, kp, kd, ki, ts):
        self.__kp = kp
        self.__kd = kd / ts
        self.__ki = ki * ts
        self.__ts = ts
        self.__error_previous = None
        self.__sum_errors = 0

    def control(self, y, set_point=0.):
        error = set_point - y
        u = self.__kp * error

        if self.__error_previous is not None:
            u += self.__kd * (error - self.__error_previous)

        u += self.__ki * self.__sum_errors
        self.__error_previous = error
        self.__sum_errors += error

        return u


murphy = Car(y_position=0.5)
t_sampling = 0.01
pid = PidController(0.25, 0.08, 0., t_sampling)

y_cache = np.array([murphy.get_y()])
for t in range(2000):
    steering_angle = pid.control(murphy.get_y())
    murphy.move(steering_angle, t_sampling)
    y_cache = np.vstack((y_cache, [murphy.get_y()]))

t_span = t_sampling * np.arange(2001)
plt.plot(y_cache)
plt.grid()
plt.xlabel('Time (s)')
plt.ylabel('y (m)')
plt.show()