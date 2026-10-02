### --- Utility Functions --- ###

def normalize(values):
    vmin, vmax = min(values), max(values)
    if vmax == vmin:
        return [0 for _ in values]
    return [(v - vmin) / (vmax - vmin) for v in values]