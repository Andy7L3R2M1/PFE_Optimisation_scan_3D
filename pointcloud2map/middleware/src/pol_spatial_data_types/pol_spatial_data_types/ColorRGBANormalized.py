from std_msgs.msg import ColorRGBA


class ColorRGBANormalized:
    def __init__(self, color: ColorRGBA):
        self.r: int = int(color.r * 255)
        self.g: int = int(color.g * 255)
        self.b: int = int(color.b * 255)
        self.a: int = int(color.a * 255)

    def __str__(self):
        return f"R: {self.r}, G: {self.g}, B: {self.b}, A: {self.a}"
