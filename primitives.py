import numpy as np


class RectangularRegion:
    def __init__(self, minx, maxx):
        self.minx = np.array(minx)
        self.maxx = np.array(maxx)
        self.dims = self.maxx - self.minx
        self.vertices = [
            [minx[0], minx[1]],
            [minx[0], maxx[1]],
            [maxx[0], minx[1]],
            [maxx[0], maxx[1]],
        ]
    
    def discretize(self, resolution):
        return np.mgrid[
            self.minx[0] : self.maxx[0] : resolution,
            self.minx[1] : self.maxx[1] : resolution,
        ]
    
    def distance_from_boundary(self, p):
        return min(
            abs(p[0] - self.minx[0]),
            abs(p[0] - self.maxx[0]),
            abs(p[1] - self.minx[1]),
            abs(p[1] - self.maxx[1])
        )


class Robot:
    def __init__(self, x):
        self.x = np.array(x)

    def __repr__(self):
        return f"Robot([{self.x[0]:.3f}, {self.x[1]:.3f}])"