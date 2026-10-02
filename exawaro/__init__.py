"""exawaro: exposure-aware routing and behavioural simulation on urban road networks.

`car_od_randomizer` and `vehicular_od_generator_pisa` are runnable scripts, not
importable library modules: they read trajectory data at import time. Run them
with ``python -m exawaro.<name>`` rather than importing them.
"""

__version__ = "0.2.0"

__all__ = [
    "cars",
    "pedestrians",
    "utils",
]