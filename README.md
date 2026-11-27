# exawaro

Exposure-aware routing and behavioural simulation on urban road networks.

This repository contains the reference implementation of the routing framework
described in *Vehicle-Pedestrian Optimization Framework for Exposure-Aware
Routing* (Mobile Networks and Applications, 2025). The broader context is the
PhD thesis *Analysis and Routing-Based Mitigation of Vehicular Air Pollution
Exposure* (University of Pisa / KDD Lab, ISTI-CNR, 2026).

## Citation

If you use this code, please cite the journal paper:

> Gurban Aliyev and Mirco Nanni.
> *Vehicle-Pedestrian Optimization Framework for Exposure-Aware Routing.*
> **Mobile Networks and Applications**, volume 30, issues 3–4, pages 325–340,
> 2025.
> DOI: [10.1007/s11036-025-02459-4](https://doi.org/10.1007/s11036-025-02459-4)

BibTeX:

```bibtex
@article{aliyev2025exawaro,
  title   = {Vehicle-Pedestrian Optimization Framework for Exposure-Aware Routing},
  author  = {Aliyev, Gurban and Nanni, Mirco},
  journal = {Mobile Networks and Applications},
  volume  = {30},
  number  = {3--4},
  pages   = {325--340},
  year    = {2025},
  doi     = {10.1007/s11036-025-02459-4}
}
```

Related work using the same method:

```bibtex
@inproceedings{aliyev2025mdm,
  title     = {Exploiting Vehicular Data for Exposure-Aware Pedestrian Routing},
  author    = {Aliyev, Gurban and Nanni, Mirco},
  booktitle = {2025 26th IEEE International Conference on Mobile Data Management (MDM)},
  year      = {2025},
  pages     = {11--19},
  doi       = {10.1109/MDM65600.2025.00022}
}

@phdthesis{aliyev2026exawaro,
  title = {Analysis and Routing-Based Mitigation of Vehicular Air Pollution Exposure},
  author = {Aliyev, Gurban},
  school = {University of Pisa},
  year = {2026},
  note = {National PhD Program in AI for Society, KDD Lab, ISTI-CNR}
}
```

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

## Limitations

- **The reported results are not empirical validation.** They come from
  constrained-route scenarios over the real OpenStreetMap network of Pisa,
  using a static emission map derived from real GPS vehicle trajectories and a
  generated pedestrian OD demand. The routing policy and the response to it are
  modelled, not observed. The percentage reductions describe modelled behaviour
  under those assumptions, not measured reductions in a real population.
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

## Repository contents

| File | Description |
|---|---|
| `exawaro/main.py` | Simulation loop and path-selection logic |
| `exawaro/cars.py` | Vehicle emission model and routing costs |
| `exawaro/pedestrians.py` | Dispersion-weighted exposure and routing |
| `exawaro/utils.py` | Normalisation helper |
| `exawaro/car_od_randomizer.py` | OD generation from raw GPS trajectories |
| `exawaro/vehicular_od_generator_pisa.py` | Bounding-box OD variant |
| `notebooks/exawaro_pedestrian.ipynb` | Full pedestrian/vehicle workflow with stored outputs |
| `notebooks/google_maps_api.ipynb` | Google Maps Places API demonstration |
| `requirements.txt` | Runtime dependencies |
| `setup.py` | Package metadata |
| `CITATION.cff` | Machine-readable citation metadata |

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

Requires Python 3.9+. The scripts target the `osmnx` 1.x API;
`requirements.txt` pins `osmnx<2.0` because the top-level `graph_from_bbox(...)`
call changed in 2.x.

## Data requirements

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
trajectory endpoints to a 200 m grid and samples two routes per trip within the
bounding box.

## Usage

Import the reusable helpers directly:

```python
from exawaro.cars import compute_emission
from exawaro.pedestrians import build_neighbor_emission_weights
from exawaro.utils import normalize
```

Or run the full simulation, which reads from `data/` and writes to `output/`:

```bash
python -m exawaro.main
```

`main.py`, `car_od_randomizer.py` and `vehicular_od_generator_pisa.py` are
runnable scripts; they read input data when executed.

## License

MIT — see [LICENSE](LICENSE).

## Contact

Gurban Aliyev — [qrb.aliyev@gmail.com](mailto:qrb.aliyev@gmail.com)
