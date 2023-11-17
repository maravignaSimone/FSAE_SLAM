# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student

import matplotlib.pyplot as plt

# defining the radius of the cones
startingConeRadius = 0.202
coneRadius = 0.162

# creating a list of tuples for the cones
# (x,y) coordinates of the cones
# linear path
carStartingPosition = (-6, 3)
startingCone = [(0,1), (0,5)]
innerCone = [(-5,5), (-2,5), (2,5), (5,5), (8,5), (11,5), (14,5), (17,5), (20,5), (23,5), (26,5), (29,5)]
outerCone = [(-5,1), (-2,1), (2,1), (5,1), (8,1), (11,1), (14,1), (17,1), (20,1), (23,1), (26,1), (29,1)]

# Plotting the cones
x_inner, y_inner = zip(*innerCone)
x_outer, y_outer = zip(*outerCone)
x_starting, y_starting = zip(*startingCone)

plt.scatter(carStartingPosition[0], carStartingPosition[1], color='green', label='Car Starting Position', marker='>')
plt.scatter(x_starting, y_starting, color='orange', label='Starting Cones')
plt.scatter(x_inner, y_inner, color='yellow', label='Inner Cones')
plt.scatter(x_outer, y_outer, color='blue', label='Outer Cones')

plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Map of the track')
plt.legend()

plt.show()

