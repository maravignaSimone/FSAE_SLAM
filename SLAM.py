# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student

# ----------------------------- #
# Libraries
# ----------------------------- #
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay
from utils import *

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
carEgoYaw = 0 # yaw angle wrt x axis in radians

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
# Defining helper functions    #
# ----------------------------- #
def distanceBetweenPoints(point1, point2):
    """
    This function returns the distance between two points.
    """
    # calculating the distance between the two points
    distance = ((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)**0.5
    return distance

def angleBetweenPoints(point1, point2):
    """
    This function returns the angle between two points.
    """
    # calculating the angle between the two points
    angle = math.atan2(point2[1] - point1[1], point2[0] - point1[0])
    return angle

def isInFov(carPosition, carYaw, conePosition, coneRadius):
    """
    This function returns True if the cone is in the FOV of the car, False otherwise.
    """
    # calculating the distance between the car and the cone
    distance = distanceBetweenPoints(carPosition, conePosition) + coneRadius
    maxAngle = carYaw + fovAngle/2
    minAngle = carYaw - fovAngle/2

    angle = angleBetweenPoints(carPosition, conePosition)
    #check if the cone is in the FOV of the car (distance smaller than the FOV distance and angle between the max and min angle)
    if distance <= fovDistance and angle <= maxAngle and angle >= minAngle:
        return True
    else:
        return False

def seenCones(carPosition, carYaw, innerCones, outerCones, startingCone):
    """
    This function returns the list of cones that are in the FOV of the car.
    """
    for cone in innerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius):
            seenInnerCones.append(cone)
    for cone in outerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius):
            seenOuterCones.append(cone)
    for cone in startingCone:
        if isInFov(carPosition, carYaw, cone, startingConeRadius):
            seenStartingCone.append(cone)

def centerLinePath(seenInnerCones, seenOuterCones, seenStartingCone):
    """
    This function find the path at the center of the road (between inner and outer cones).
    """

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
print("Starting the SLAM algorithm...")
#checking the fov function
print("Checking the isInFov function...")
print("Is the starting cone in the FOV of the car?")
print(isInFov(carEgoPosition, carEgoYaw, startingCone[0], startingConeRadius, fovAngle, fovDistance))
seenCones(carEgoPosition, carEgoYaw, innerCone, outerCone, startingCone, seenInnerCones, seenOuterCones, seenStartingCone, coneRadius, startingConeRadius, fovAngle, fovDistance)
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
plt.axis('equal')
plt.legend()
plt.show()
print("The SLAM algorithm has finished.")


# creating the lists of x and y coordinates for the cones
x_inner, y_inner = zip(*seenInnerCones)
x_outer, y_outer = zip(*seenOuterCones)

# Create a list of all cone coordinates
#cone_coordinates has to alternate between inner and outer cones (inner, outer, inner, outer, ...)
cone_coordinates = [x for y in zip(seenInnerCones, seenOuterCones) for x in y]
print(cone_coordinates)
# Perform Delaunay triangulation
tri = Delaunay(cone_coordinates)
# Remove triangles that are outside the track
simplices = list(tri.simplices)
for simplex in simplices:
    if (simplex[0] % 2 == 0 and simplex[1] % 2 == 0 and simplex[2] % 2 == 0) or (simplex[0] % 2 != 0 and simplex[1] % 2 != 0 and simplex[2] % 2 != 0):
        simplices.remove(simplex)
        
# Plotting the oriented car, with the right orientation(Yaw) and FOV
plt.scatter(carEgoPosition[0], carEgoPosition[1], color='green', label='Car')
plt.quiver(carEgoPosition[0], carEgoPosition[1], math.cos(carEgoYaw), math.sin(carEgoYaw), color='green', label='Yaw', scale=15)
plt.plot([carEgoPosition[0], carEgoPosition[0]+fovDistance*math.cos(carEgoYaw+fovAngle/2)], [carEgoPosition[1], carEgoPosition[1]+fovDistance*math.sin(carEgoYaw+fovAngle/2)], color='green', label = 'FOV')
plt.plot([carEgoPosition[0], carEgoPosition[0]+fovDistance*math.cos(carEgoYaw-fovAngle/2)], [carEgoPosition[1], carEgoPosition[1]+fovDistance*math.sin(carEgoYaw-fovAngle/2)], color='green')

# Plotting the cones
plt.scatter(x_starting, y_starting, color='orange', label='Starting Cones')
plt.scatter(x_inner, y_inner, color='yellow', label='Inner Cones')
plt.scatter(x_outer, y_outer, color='blue', label='Outer Cones')

# Plotting the Delaunay triangulation
x_cone, y_cone = zip(*cone_coordinates)
plt.triplot(x_cone, y_cone, simplices, color='red', label='Delaunay Triangulation')

# Show the plot
plt.legend()
plt.show()
