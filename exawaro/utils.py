import numpy as np

### --- Utility Functions --- ###

def edge_midpoint(geom):
    return geom.interpolate(0.5, normalized=True)

def gaussian_weight(d, sigma):
    return exp(- (d ** 2) / (2 * sigma ** 2))

def normalize(values):
    vmin, vmax = min(values), max(values)
    if vmax == vmin:
        return [0 for _ in values]
    return [(v - vmin) / (vmax - vmin) for v in values]
