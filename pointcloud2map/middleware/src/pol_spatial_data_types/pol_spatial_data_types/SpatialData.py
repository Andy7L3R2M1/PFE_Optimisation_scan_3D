from .SpatialPoint import SpatialPoint


class SpatialData:
    def __init__(self):
        self.points = []

    def register_point(self, point: SpatialPoint):
        self.points.append(point)

    def __str__(self):
        return f"Points: {self.points}"
