import time
from .ColorRGBANormalized import ColorRGBANormalized


class SpatialPoint:
    def __init__(self, x: int, y: int, z: int, color: ColorRGBANormalized):
        self.x = x  # in centimeters
        self.y = y  # in centimeters
        self.z = z  # in centimeters
        self.color = color  # normalized RGBA color
        self.timestamp = time.time()  # in seconds

    def __str__(self):
        return f"X: {self.x}, Y: {self.y}, Z: {self.z}, Color: {self.color}, Timestamp: {self.timestamp}"
