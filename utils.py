# Authors: Simone Maravigna, Francesco Marotta
# Project: SLAM in Formula Student
# Description: This file contains some helper functions.

# ----------------------------- #
# Libraries
# ----------------------------- #
import math
from scipy.spatial import Delaunay
import numpy as np
from scipy.interpolate import make_interp_spline
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

def angleBetweenTwoVectors(vector1, vector2):
    """
    This function returns the angle between two vectors.
    """
    # calculating the angle between the two vectors
    angle = math.acos(np.dot(vector1, vector2)/(np.linalg.norm(vector1)*np.linalg.norm(vector2)))
    return angle
def angleBetweenCarAndPoint(carPosition, carYaw, point):
    """
    This function returns the angle between the car and a point.
    """
    # calculating the angle between the car and the point
    carVector = np.array([math.cos(carYaw), math.sin(carYaw)])
    pointVector = np.array([point[0]-carPosition[0], point[1]-carPosition[1]])
    angle = angleBetweenTwoVectors(carVector, pointVector)
    return angle

def isInFov(carPosition, carYaw, conePosition, coneRadius, fovAngle, fovDistance):
    """
    This function returns True if the cone is in the FOV of the car, False otherwise.
    """
    # calculating the distance between the car and the cone
    distance = distanceBetweenPoints(carPosition, conePosition) + coneRadius
    maxAngle = fovAngle/2
    minAngle = -fovAngle/2
    # calculating the angle between the car and the cone
    angle = angleBetweenCarAndPoint(carPosition, carYaw, conePosition)
    #check if the cone is in the FOV of the car (distance smaller than the FOV distance and angle between the max and min angle)
    if distance <= fovDistance and angle <= maxAngle and angle >= minAngle:
        return True
    else:
        return False

def seenCones(carPosition, carYaw, innerCones, outerCones, startingCone, seenInnerCones, seenOuterCones, seenStartingCone, coneRadius, startingConeRadius, fovAngle, fovDistance, inverse_transformation_matrix):
    """
    This function returns the list of cones that are in the FOV of the car.
    """
    for cone in innerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius, fovAngle, fovDistance):
            noisy_cone = np.array(cone) + np.random.normal(scale=0, size=len(cone))
            seenInnerCones.append(pointFromWorldToCamera(noisy_cone, inverse_transformation_matrix))
    for cone in outerCones:
        if isInFov(carPosition, carYaw, cone, coneRadius, fovAngle, fovDistance):
            noisy_cone = np.array(cone) + np.random.normal(scale=0, size=len(cone))
            seenOuterCones.append(pointFromWorldToCamera(noisy_cone, inverse_transformation_matrix))
    for cone in startingCone:
        if isInFov(carPosition, carYaw, cone, startingConeRadius, fovAngle, fovDistance):
            noisy_cone = np.array(cone) + np.random.normal(scale=0, size=len(cone))
            seenStartingCone.append(pointFromWorldToCamera(noisy_cone, inverse_transformation_matrix))

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
    # Perform Delaunay triangulation
    tri = Delaunay(cone_coordinates, qhull_options="QJ Qa Qs")
    # Remove triangles that are outside the track
    simplices = tri.simplices
    simplices_to_save = []
    for n, simplex in enumerate(simplices):
        if not((simplex[0] % 2 == 0 and simplex[1] % 2 == 0 and simplex[2] % 2 == 0) or (simplex[0] % 2 != 0 and simplex[1] % 2 != 0 and simplex[2] % 2 != 0)):
            simplices_to_save.append(simplex)
    return simplices_to_save, cone_coordinates

def findMidPoint(point1, point2):
    """
    This function returns the midpoint between two points.
    """
    # calculating the midpoint between the two points
    midPoint = ((point1[0]+point2[0])/2, (point1[1]+point2[1])/2)
    return midPoint

def oddEvenPoints(triangle):
    """ 
    This function returns the list of the points of the triangle that have different parity.
    """
    points = []
    if (triangle[1] % 2 == 0 and triangle[2] % 2 != 0) or (triangle[1] % 2 != 0 and triangle[2] % 2 == 0):
        points.append(tuple([triangle[1], triangle[2]]))
    if (triangle[0] % 2 == 0 and triangle[1] % 2 != 0) or (triangle[0] % 2 != 0 and triangle[1] % 2 == 0):
        points.append(tuple([triangle[0], triangle[1]]))
    if (triangle[0] % 2 == 0 and triangle[2] % 2 != 0) or (triangle[0] % 2 != 0 and triangle[2] % 2 == 0):
        points.append(tuple([triangle[0], triangle[2]]))
    return points

def findTriangulationMidPoints(simplices, cone_coordinates):
    """
    This function returns the list of the mid points of the triangles of the triangulation.
    """
    midPoints = set()
    for triangle in simplices:
        #find the points of the triangle that have different parity
        points = oddEvenPoints(triangle)
        for point in points:
            midPoints.add(findMidPoint(cone_coordinates[point[0]], cone_coordinates[point[1]]))
    return midPoints

def computeTransformationMatrixes(carEgoPosition, carEgoYaw):
    """
    This function returns the transformation matrix from the car reference frame to the world reference frame.
    """
    transformation_matrix = np.array([[math.cos(carEgoYaw), -math.sin(carEgoYaw), carEgoPosition[0]],
                                      [math.sin(carEgoYaw), math.cos(carEgoYaw), carEgoPosition[1]],
                                      [0, 0, 1]])
    inverse_transformation_matrix = np.linalg.inv(transformation_matrix)
    return transformation_matrix, inverse_transformation_matrix
    
def computeSpline(points):
    """
    This function returns the spline of a passed set of points.
    """
    x= [x[0] for x in points]
    y= [x[1] for x in points]
    phi = np.linspace(0, 2*np.pi, points.__len__())
    spl = make_interp_spline(phi, np.c_[x, y], k=2)
    phi_new = np.linspace(0, 2*np.pi, 300)
    x_new, y_new = spl(phi_new).T
    return x_new, y_new

def pointFromCameraToWorld(point, transformation_matrix):
    """
    This function returns the point in the world reference frame.
    """
    point_homogeneous = np.array([point[0], point[1], 1])
    point_world = np.dot(transformation_matrix, point_homogeneous)
    point_world = point_world[:2]
    return point_world

def pointFromWorldToCamera(point, inverse_transformation_matrix):
    """
    This function returns the point in the camera reference frame.
    """
    point_homogeneous = np.array([point[0], point[1], 1])
    point_camera = np.dot(inverse_transformation_matrix, point_homogeneous)
    point_camera = point_camera[:2]
    return point_camera