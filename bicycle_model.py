from math import sin, cos

class BicycleModel:
    def __init__(self, delta_time = 0.1):
        self.delta_time = delta_time

    # update step: given the current state of the car, the velocity and the curvature, compute the new state
    def update(self, x, y, yaw, velocity, curvature):
        # discretized bicycle model equations
        new_x = x + self.delta_time * velocity * cos(yaw)
        new_y = y + self.delta_time * velocity * sin(yaw)
        new_yaw = yaw + self.delta_time * velocity * curvature

        return (new_x, new_y), new_yaw