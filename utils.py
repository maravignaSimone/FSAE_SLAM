# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student
# Description: This file contains some helper functions.

# ----------------------------- #
# Libraries
# ----------------------------- #
import math
from scipy.spatial import Delaunay
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

def isInFov(carPosition, carYaw, conePosition, coneRadius, fovAngle, fovDistance):
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

def seenCones(carPosition, carYaw, innerCones, outerCones, startingCone, seenInnerCones, seenOuterCones, seenStartingCone, coneRadius, startingConeRadius, fovAngle, fovDistance):
    """
    This function returns the list of cones that are in the FOV of the car.
    """
    for cone in innerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius, fovAngle, fovDistance):
            seenInnerCones.append(cone)
    for cone in outerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius, fovAngle, fovDistance):
            seenOuterCones.append(cone)
    for cone in startingCone:
        if isInFov(carPosition, carYaw, cone, startingConeRadius, fovAngle, fovDistance):
            seenStartingCone.append(cone)

def centerLinePath(seenInnerCones, seenOuterCones, seenStartingCone):
    """
    This function find the path at the center of the road (between inner and outer cones).
    """

def triangulation(seenInnerCones, seenOuterCones, seenStartingCone):
    """
    This function returns the simplices of the delaunay triangulation of the cones.
    """
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
    return simplices, cone_coordinates
            