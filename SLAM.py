# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student

# ----------------------------- #
# Libraries
# ----------------------------- #
import matplotlib.pyplot as plt
from utils import *
from scipy.interpolate import make_interp_spline
import numpy as np
from scipy import interpolate
# ----------------------------- #
# Defining global variables
# ----------------------------- #
startingConeRadius = 0.202 # measure of the starting cone radius in meters
coneRadius = 0.162 # measure of the cone radius in meters

carStartingPosition = (3, 3) # coordinates of the starting position of the car
startingCone = [(0,1), (0,5)] # coordinates of the starting cone

fovAngle = 2*math.pi/3 # FOV measured in radian
fovDistance = 10 # FOV distance measured in meters 

carEgoPosition = carStartingPosition # coordinates of the car
carEgoYaw = -np.pi/12# yaw angle wrt x axis in radians
transformation_matrix = np.array([
    [np.cos(carEgoYaw), -np.sin(carEgoYaw), carEgoPosition[0]],
    [np.sin(carEgoYaw), np.cos(carEgoYaw), carEgoPosition[1]],
    [0, 0, 1]
])
inverse_transformation_matrix = np.linalg.inv(transformation_matrix)
# coordinates of the cones
innerCone = [(-5,5), (-2,5), (2,5), (5,5), (8,3), (11,5), (14,5), (17,5), (20,5), (23,5), (26,5), (29,5)]
outerCone = [(-5,1), (-2,1), (2,1), (5,1), (8,-1), (11,1), (14,1), (17,1), (20,1), (23,1), (26,1), (29,1)]
# lists of the cones that are in the FOV of the car
seenInnerCones = []
seenOuterCones = []
seenStartingCone = []
# list of the points of the trajectory
trajectoryPoints = []



# ----------------------------- #
# Main function
# ----------------------------- #

# ----------------------------- #
# Plotting the map             #
# ----------------------------- #
# creating the lists of x and y coordinates for the cones
x_inner, y_inner = zip(*innerCone)
x_outer, y_outer = zip(*outerCone)
x_starting, y_starting = zip(*startingCone)

# Plotting the oriented car, with the right orientation(Yaw) anf FOV
plt.scatter(carEgoPosition[0], carEgoPosition[1], color='green', label='Car')
plt.quiver(carEgoPosition[0], carEgoPosition[1], math.cos(carEgoYaw), math.sin(carEgoYaw), color='green', label='Yaw', scale=15)
plt.plot([carEgoPosition[0], carEgoPosition[0]+fovDistance*math.cos(carEgoYaw+fovAngle/2)], [carEgoPosition[1], carEgoPosition[1]+fovDistance*math.sin(carEgoYaw+fovAngle/2)], color='green', label = 'FOV')
plt.plot([carEgoPosition[0], carEgoPosition[0]+fovDistance*math.cos(carEgoYaw-fovAngle/2)], [carEgoPosition[1], carEgoPosition[1]+fovDistance*math.sin(carEgoYaw-fovAngle/2)], color='green')

# Plotting the cones
plt.scatter(x_starting, y_starting, color='orange', label='Starting Cones')
plt.scatter(x_inner, y_inner, color='yellow', label='Inner Cones')
plt.scatter(x_outer, y_outer, color='blue', label='Outer Cones')

plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Map of the track')
plt.legend()
plt.show()
# ----------------------------- #
seenCones(carEgoPosition, carEgoYaw, innerCone, outerCone, startingCone, seenInnerCones, seenOuterCones, seenStartingCone, coneRadius, startingConeRadius, fovAngle, fovDistance, inverse_transformation_matrix)
print("The cones in the FOV of the car are:")
print("Starting cone: ", seenStartingCone)
print("Inner cones: ", seenInnerCones)
print("Outer cones: ", seenOuterCones)
#plotting the seen cones
if len(seenStartingCone) > 0:
    x_seenStarting, y_seenStarting = zip(*seenStartingCone)
    plt.scatter(x_seenStarting, y_seenStarting, color='orange', label='Seen Starting Cones')
if len(seenInnerCones) > 0:
    x_seenInner, y_seenInner = zip(*seenInnerCones)
    plt.scatter(x_seenInner, y_seenInner, color='yellow', label='Seen Inner Cones')
if len(seenOuterCones) > 0:
    x_seenOuter, y_seenOuter = zip(*seenOuterCones)
    plt.scatter(x_seenOuter, y_seenOuter, color='blue', label='Seen Outer Cones')
plt.scatter(0, 0, color='green', label='Car')
plt.axis('equal')
plt.legend()
plt.title('Seen cones in CRF')
plt.show()


# creating the lists of x and y coordinates for the cones
x_inner, y_inner = zip(*seenInnerCones)
x_outer, y_outer = zip(*seenOuterCones)

simplices, cone_coordinates = triangulation(seenInnerCones, seenOuterCones, seenStartingCone)
midPoints = findTriangulationMidPoints(simplices, cone_coordinates)
print("Mid", midPoints)
midPoints = sorted(midPoints, key=lambda x: x[0])
x_mid = [x[0] for x in midPoints]
y_mid = [x[1] for x in midPoints]
phi = np.linspace(0, 2*np.pi, midPoints.__len__())
spl = make_interp_spline(phi, np.c_[x_mid, y_mid], k=2)
phi_new = np.linspace(0, 2*np.pi, 100)
x_new, y_new = spl(phi_new).T



# Plotting the oriented car, with the right orientation(Yaw) and FOV
plt.scatter(0, 0, color='green', label='Car')
plt.quiver(0, 0, math.cos(0), math.sin(0), color='green', label='Yaw', scale=15)
plt.plot([0, fovDistance*math.cos(0+fovAngle/2)], [0, fovDistance*math.sin(0+fovAngle/2)], color='green', label = 'FOV')
plt.plot([0, fovDistance*math.cos(0-fovAngle/2)], [0, fovDistance*math.sin(0-fovAngle/2)], color='green')

# Plotting the cones
#plt.scatter(x_starting, y_starting, color='orange', label='Starting Cones')
plt.scatter(x_inner, y_inner, color='yellow', label='Inner Cones')
plt.scatter(x_outer, y_outer, color='blue', label='Outer Cones')

# Plotting the Delaunay triangulation
x_cone, y_cone = zip(*cone_coordinates)
plt.triplot(x_cone, y_cone, simplices, color='red', label='Delaunay Triangulation')

# Plotting the midpoints
x_mid, y_mid = zip(*midPoints)
plt.scatter(x_mid, y_mid, color='black', label='Midpoints')
plt.plot(x_new, y_new, color='black', label='Center Line')
plt.axis('equal')
# Show the plot
plt.legend()
plt.show()
