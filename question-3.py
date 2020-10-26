import numpy as np
import matplotlib.pyplot as plt
from Car import Car


# Define the 2 dimensional array of Car objects
car = [[Car(velocity=1), Car(velocity=2), Car(velocity=5), Car(velocity=10)],
       [Car(length=1), Car(length=2), Car(length=5), Car(length=10)],
       [Car(), Car(pose=np.deg2rad(90)), Car(pose=np.deg2rad(180)), Car(pose=np.deg2rad(270))]]

# Declare the global variables
dt = 500  # The total time of the simulation
car_steering_angle = np.deg2rad(-5)  # The steering angle of the car, which is a constant
num_points = 1000  # The resolution of the graph

# Declare the array to store all of the car trajectories
car_trajectory = []

# Simulation of the car with different initial values of v, L, and theta
for j in range(3):
    # Add an empty array within the car trajectory array to aid plotting the graphs later
    car_trajectory.append([])

    for i in range(4):
        # Append the solution of the system dynamics to the car trajectory array
        car_trajectory[j].append(car[j][i].move(car_steering_angle, dt, num_points))

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
        plt.savefig('figures\\question_3_a.svg', format='svg')  # Save the graph as a .svg file
    if j == 1:
        plt.savefig('figures\\question_3_b.svg', format='svg')  # Save the graph as a .svg file
    if j == 2:
        plt.savefig('figures\\question_3_c.svg', format='svg')  # Save the graph as a .svg file

    plt.show()
