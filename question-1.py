import numpy as np
import matplotlib.pyplot as plt
from Car import Car


# Initialise the Car object
car = Car(y_position=0.3, pose=np.deg2rad(5))

# Declare global variables
car_steering_angle = np.deg2rad(-2)
dt = 2
num_points = 1000  # The resolution of the graph
car_trajectory = car.move(car_steering_angle, dt, num_points)

# Question 1.1 - Plot x position (m) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[0].T)
plt.xlabel('Time (s)')
plt.ylabel('x Position (m)')
plt.grid()
plt.savefig('figures\\question_1_1_a.svg', format='svg')
plt.show()

# Question 1.1 - Plot y position (m) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[1].T)
plt.xlabel('Time (s)')
plt.ylabel('y Position (m)')
plt.grid()
plt.savefig('figures\\question_1_1_b.svg', format='svg')
plt.show()

# Question 1.1 - Plot theta (rad) against time (s)
plt.plot(car_trajectory.t, car_trajectory.y[2].T)
plt.xlabel('Time (s)')
plt.ylabel('Theta (rad)')
plt.grid()
plt.savefig('figures\\question_1_1_c.svg', format='svg')
plt.show()

# Question 1.2 - Plot y position (m) against x position (m)
plt.plot(car_trajectory.y[0], car_trajectory.y[1].T)
plt.xlabel('x Position (m)')
plt.ylabel('y Position (m)')
plt.grid()
plt.savefig('figures\\question_1_2.svg', format='svg')
plt.show()
