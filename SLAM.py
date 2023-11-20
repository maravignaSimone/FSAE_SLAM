# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student

# ----------------------------- #
# Importing the needed modules #
# ----------------------------- #
import matplotlib.pyplot as plt
import math

# ----------------------------- #
# Defining global variables    #
# ----------------------------- #
fovAngle = 180
fovDistance = 10

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

def isInFov(carPosition, carOrientation, conePosition, coneRadius):
    """
    This function returns True if the cone is in the FOV of the car, False otherwise.
    """
    # calculating the distance between the car and the cone
    distance = distanceBetweenPoints(carPosition, conePosition)

    # calculating the angle between the car and the cone
    angle = angleBetweenPoints(carPosition, conePosition) - carOrientation
    print(distance, angle)
    if distance <= fovDistance and abs(angle) <= fovAngle:
        return True
    return False

# ----------------------------- #
# Defining the main function   #
# ----------------------------- #

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

# defining car position and orientation
carEgoPosition = carStartingPosition
#carEgoOrientation = right forward
carEgoOrientation = math.pi

# creating a list of tuples for the visible cones
orangeVisibleCones = []
yellowVisibleCones = []
blueVisibleCones = []

# Plotting the cones
x_inner, y_inner = zip(*innerCone)
x_outer, y_outer = zip(*outerCone)
x_starting, y_starting = zip(*startingCone)

#plotting the car oriented in the right direction
plt.plot([carEgoPosition[0], carEgoPosition[0] + 0.5*math.cos(carEgoOrientation)], [carEgoPosition[1], carEgoPosition[1] + 0.5*math.sin(carEgoOrientation)], color='black')

plt.scatter(x_starting, y_starting, color='orange', label='Starting Cones')
plt.scatter(x_inner, y_inner, color='yellow', label='Inner Cones')
plt.scatter(x_outer, y_outer, color='blue', label='Outer Cones')

plt.axis('equal')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Map of the track')
plt.legend()

#plotting the fov
plt.plot([carEgoPosition[0], carEgoPosition[0] + fovDistance*math.cos(carEgoOrientation + fovAngle)], [carEgoPosition[1], carEgoPosition[1] + fovDistance*math.sin(carEgoOrientation + fovAngle)], color='red')
plt.plot([carEgoPosition[0], carEgoPosition[0] + fovDistance*math.cos(carEgoOrientation - fovAngle)], [carEgoPosition[1], carEgoPosition[1] + fovDistance*math.sin(carEgoOrientation - fovAngle)], color='red')

plt.show()

# ----------------------------- #
print("Starting the SLAM algorithm...")
#checking the fov function
print("Checking the isInFov function...")
print("Is the starting cone in the FOV of the car?")
print(isInFov(carEgoPosition, carEgoOrientation, startingCone[0], startingConeRadius))