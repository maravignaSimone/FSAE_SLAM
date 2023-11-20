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
fovAngle = math.pi # measures in radian the FOV
fovDistance = 10 # measures in meters the FOV

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
carEgoYaw = 0 # 0 radian wrt x axis

# creating a list of tuples for the visible cones
orangeVisibleCones = []
yellowVisibleCones = []
blueVisibleCones = []

# Plotting the cones
x_inner, y_inner = zip(*innerCone)
x_outer, y_outer = zip(*outerCone)
x_starting, y_starting = zip(*startingCone)


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
print(isInFov(carEgoPosition, carEgoYaw, startingCone[0], startingConeRadius))