# exawaro

Exposure-aware routing and behavioural simulation on urban road networks.

This repository contains the code underlying my PhD thesis, *"Analysis and
Routing-Based Mitigation of Vehicular Air Pollution Exposure"* (National PhD
Program in AI for Society, University of Pisa / KDD Lab, ISTI-CNR, 2026).

The question it addresses: **vehicles choose the fastest route; pedestrians
breathe whatever that route emits.** If route choice accounts for the
population living alongside a road, the optimal path stops being the fastest
one.

---

## What the code does

The simulation runs on road networks downloaded from OpenStreetMap via
`osmnx`, over a bounding box in central Pisa, and couples two agent groups
across a shared emission field.

**Vehicles** are routed on a `drive_service` graph. Emissions per edge follow a
speed-dependent polynomial cost function (`cars.compute_emission`), evaluated
per traversal and accumulated into an emission map each simulation step.

**Pedestrians** are routed on a `walk` graph. Rather than only accounting for
emissions on the road they walk beside, `pedestrians.build_neighbor_emission_weights`
buffers every pedestrian edge and distributes nearby road emissions onto it
with a Gaussian decay in midpoint distance. This models dispersion: a walker is
exposed to traffic on nearby streets, not just the one adjacent to the pavement.
Exposure is then normalised per metre and scaled by traversal time.

**Both** populations then choose paths by sweeping a weight `w` geometrically
from `1.0` down to `0.018` (ten values, `0.8^k` for even `k`), building a scalar
cost `w * time + (1 - w) * exposure`, and selecting among the resulting paths
by hypervolume contribution against an ideal point derived from the sweep. A
high `w` recovers fastest-time routing; a low `w` prioritises clean air at the
cost of time.

Two selection modes are implemented:

| Mode | Steps | Behaviour |
|------|-------|-----------|
| `local` | 5 | Ideal point derived per OD pair, so each traveller optimises against their own achievable range |
| `global` | 7 | A single network-wide ideal point derived from metrics aggregated across all OD pairs |

Results are written as per-step CSVs (route assignments, emission maps,
population maps, and a summary of time and exposure deltas against fastest-path
baselines) plus a pickled run log.

---

## Honest limitations

These matter more than the feature list, and I would rather state them here
than have a reviewer find them.

- **The reported results are not empirical validation.** They come from
  constrained-route scenarios over a synthetic city model. The percentage
  reductions describe modelled behaviour under those assumptions, not measured
  reductions in a real population. Figures quoted in the thesis carry this
  caveat explicitly.
- **The emission model is a speed-based polynomial approximation**, not a
  measured or on-road emissions inventory. It captures the qualitative
  relationship between speed and emissions, not absolute quantities.
- **No behavioural calibration.** Agents respond to cost as rational optimisers.
  Real route choice is heterogeneous, habitual, and partly captive. Sensitivity
  analysis over the weight sweep is the only robustness check applied.
- **Population density is not person-resolved.** Exposure aggregates over
  network edges; it does not track which specific individuals are affected.
- **Single city, single network.** All experiments use one bounding box in Pisa.
  Transferability to other cities is untested.
- **Pedestrian dispersion uses a Gaussian kernel with hand-set parameters**
  (100 m buffer, 50 m sigma). These were not fitted to measured concentration
  fields.

---

## Installation

```bash
git clone https://github.com/baygaliyev/exawaro.git
cd exawaro
python -m venv .venv
# Linux/macOS
source .venv/bin/activate
# Windows
.venv\Scripts\activate

pip install -r requirements.txt
pip install -e .
```

Requires Python 3.9+.

## Input data

Input data is **not** included in this repository. The `data/` directory is
git-ignored, and the trajectory data it expects is derived from a commercial
GPS trajectory provider under terms that do not permit redistribution.

You need to supply origin–destination pairs as CSVs with at least
`origin_node` and `destination_node` columns, where nodes are OSM node IDs from
the same bounding box:

```
data/
  noise_od_pairs_pisa.csv
  workers_students_od_pairs_pisa.csv
  tourist_5000_od_pairs_pisa.csv
  cars_od_pairs_pisa_2526_routes.csv
```

To regenerate the vehicular OD set from raw GPS traces, place
`italy_trajectories_week_22.csv` (columns `tid`, `datetime`, `lat`, `lng`) in
`data/trajectories/` and run `python -m exawaro.car_od_randomizer`. This snaps
trajectory endpoints to a 200 m grid and samples routes within the bounding box.

## Usage

```python
from exawaro.cars import compute_emission
from exawaro.pedestrians import build_neighbor_emission_weights
from exawaro.utils import normalize
```

Or run the full simulation, which reads from `data/` and writes to `output/`:

```bash
python -m exawaro.main
```

## Project layout

```
exawaro/
  main.py                              simulation loop and path-selection logic
  cars.py                              vehicle emission model and routing costs
  pedestrians.py                       dispersion-weighted exposure and routing
  utils.py                             normalisation helpers
  car_od_randomizer.py                 OD generation from raw GPS trajectories
  vehicular_od_generator_pisa.py       bounding-box OD variant
```

## Citation

If you use this code, please cite the thesis:

```bibtex
@phdthesis{aliyev2026exposure,
  title     = {Analysis and Routing-Based Mitigation of Vehicular Air Pollution Exposure},
  author    = {Aliyev, Gurban},
  school    = {University of Pisa},
  year      = {2026},
  note      = {National PhD Program in AI for Society, KDD Lab, ISTI-CNR}
}
```

Related published work using this method:

```bibtex
@inproceedings{aliyev2025mdm,
  title     = {Exploiting Vehicular Data for Exposure-Aware Pedestrian Routing},
  author    = {Aliyev, Gurban and Nanni, Mirco},
  booktitle = {2025 26th IEEE International Conference on Mobile Data Management (MDM)},
  year      = {2025},
  pages     = {11--19},
  doi       = {10.1109/MDM65600.2025.00022}
}

@article{aliyev2025vpof,
  title  = {Vehicle-Pedestrian Optimization Framework for Exposure-Aware Routing},
  author = {Aliyev, Gurban and Nanni, Mirco},
  year   = {2025},
  doi    = {10.1007/s11036-025-02459-4}
}
```

## Licence

MIT — see [LICENSE](LICENSE).

## Contact

Gurban Aliyev — [qrb.aliyev@gmail.com](mailto:qrb.aliyev@gmail.com)