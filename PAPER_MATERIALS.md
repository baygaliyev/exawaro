# Paper figures and results

This file maps the figures and tables used in the MDM 2025 and MONET 2025
papers to material included here. Notebook cell numbers refer to
`notebooks/exawaro_pedestrian.ipynb` as released in v0.2.2.

Status labels:

- **stored figure** — exact or near-exact figure file is included in `figures/`.
- **notebook output** — a stored output reproducing the figure is in the notebook.
- **result only** — numeric result is available, but not as a standalone figure.
- **gap** — generation code or simulation output was not found in this repository.

## MDM 2025

Paper: *Exploiting Vehicular Data for Exposure-Aware Pedestrian Routing*,
DOI `10.1109/MDM65600.2025.00022`. The paper has no numbered tables.

| Paper item | Material in this repository | Status |
|---|---|---|
| Fig. 1 — Pareto fronts for three OD pairs | `figures/pareto_front_three_pairs.pdf`; notebook cell 94 | stored figure + notebook output |
| Fig. 2 — CO2 concentration map | `figures/concentration_map_comparison.pdf` | stored figure; generation code is outside this notebook (IntSys/EIDE pipeline) |
| Fig. 3 — walking time and personal exposure maps | `figures/exawaro/time_and_road_exposure_map_separate_normalization.pdf`; notebook cell 81 | stored figure + notebook output |
| Fig. 4 — OD origins, destinations, and example pairs | `figures/origins_1k.pdf`, `figures/destinations_1k.pdf`, `figures/od_pairs_example.pdf`; notebook cells 48–50 | stored figures + notebook outputs |
| Fig. 5 — fastest-route duration and exposure distributions | `figures/exawaro/distributions_formatted.pdf`; notebook cell 73 | stored figure + notebook output |
| Fig. 6 — routes over concentration map | `figures/exawaro/logged_concentration_map_and_routes.pdf`; notebook cells 35–36 | stored figure + notebook output |
| Fig. 7 — least-cost time/exposure histograms | `figures/histograms-v2.pdf`; notebook cell 101 | stored figure + notebook output |
| Fig. 8 — Jaccard similarity boxplots | `figures/jaccard_boxplots.pdf`; notebook cell 107 | stored figure + notebook output |
| Fig. 9 — average Pareto front | `figures/exawaro/pareto_avg_differences_v2.pdf`; notebook cell 118 | stored figure + notebook output; primary reused figure |
| Fig. 10 — median Pareto front | `figures/pareto_med_differences.pdf`; notebook cell 97 | stored figure + notebook output |
| Fig. 11 — local vs global weighting | `figures/global_vs_local_scatter.pdf`; notebook cell 127 | stored figure + notebook output |
| Fig. 12 — road-usage differences | `figures/road_usage_difference.pdf`; notebook cell 135 | stored figure + notebook output |

## MONET 2025

Paper: *Vehicle-Pedestrian Optimization Framework for Exposure-Aware Routing*,
DOI `10.1007/s11036-025-02459-4`. Figures 1–4 are explicitly credited to the MDM
paper in the MONET caption.

| Paper item | Material in this repository | Status |
|---|---|---|
| Fig. 1 — walking time and exposure maps | `figures/exawaro/time_and_road_exposure_map_separate_normalization.pdf`; notebook cell 81 | stored figure + notebook output; reused from MDM Fig. 3 |
| Fig. 2 — fastest-route distributions | `figures/exawaro/distributions_formatted.pdf`; notebook cell 73 | stored figure + notebook output; reused from MDM Fig. 5 |
| Fig. 3 — routes over concentration map | `figures/exawaro/logged_concentration_map_and_routes.pdf`; notebook cells 35–36 | stored figure + notebook output; reused from MDM Fig. 6 |
| Fig. 4 — average Pareto front | `figures/exawaro/pareto_avg_differences_v2.pdf`; notebook cell 118 | stored figure + notebook output; reused from MDM Fig. 9 |
| Table 1 — simulation-step percentage changes | `paper_materials/baseline_comparison.docx`; values also in the thesis | result only; no standalone table output in the notebook |
| Fig. 5 — global road-usage evolution | `figures/road_usage_global.pdf` | stored figure; no notebook plotting cell found |
| Fig. 6 — global density/emission scatterplots | `figures/scatter_emis_pop_global.pdf` | stored figure; no notebook plotting cell found |
| Fig. 7 — global exposure maps and scatter | `figures/map_exposure_global.pdf`, `figures/scatter_exposure_change_global.pdf` | stored figures; no notebook plotting cell found |
| Fig. 8 — local road-usage evolution | `figures/road_usage_local.pdf` | stored figure; no notebook plotting cell found |
| Fig. 9 — local density/emission scatterplots | `figures/scatter_emis_pop_local.pdf` | stored figure; no notebook plotting cell found |
| Fig. 10 — local exposure maps and scatter | `figures/map_exposure_local.pdf`, `figures/scatter_exposure_change_local.pdf` | stored figures; no notebook plotting cell found |

## Known gaps

- The concentration map used in MDM Fig. 2 is produced by the earlier
  exposure-estimation pipeline, archived separately as EIDE (INTSYS 2024),
  rather than by the exawaro notebook.
- The simulation runner writes step-level CSV/PKL outputs, but no stored
  simulation outputs are included here. Restricted trajectory inputs and
  derived OD CSVs cannot be redistributed.
- The notebooks contain scenario reruns, an interrupted run (cell 54), and a
  failed projection attempt (cell 154). These are annotated as exploratory and
  are not paper figures.
- Some stored MONET figures are labelled `Final Step` while the paper captions
  name the concrete step number. The archived figures are the versions used in
  the thesis; exact paper-pixel identity was not independently proven.
