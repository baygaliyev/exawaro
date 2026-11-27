"""Small numeric helpers used by the exposure model."""


def normalize(values):
    """Scale values to [0, 1]; return zeros when all values are equal."""
    vmin, vmax = min(values), max(values)
    if vmax == vmin:
        return [0 for _ in values]
    return [(v - vmin) / (vmax - vmin) for v in values]
