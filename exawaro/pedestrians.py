from collections import defaultdict
from math import exp
import networkx as nx
import numpy as np
from shapely.geometry import LineString
from shapely.ops import split
from shapely.strtree import STRtree
from shapely import prepared

from utils import normalize

def edge_midpoint(geom):
    return geom.interpolate(0.5, normalized=True)

def gaussian_weight(d, sigma):
    return exp(- (d ** 2) / (2 * sigma ** 2))


def build_neighbor_emission_weights(graph, buffer_radius=100, sigma=50):
    """
    For each edge, find portions of other edges that intersect its buffer,
    and assign them weights based on midpoint distance (Gaussian decay).
    
    Returns:
        neighbor_weights: dict[(u,v,key)] = list of ((nu, nv, nk), weight)
    """
    edge_geoms = {}
    edge_midpoints = {}
    edge_geom_list = []
    edge_keys_list = []

    # Step 1: build edge geometry and key lists
    for u, v, key, data in graph.edges(keys=True, data=True):
        if 'geometry' in data:
            geom = data['geometry']
        else:
            p1 = (graph.nodes[u]['x'], graph.nodes[u]['y'])
            p2 = (graph.nodes[v]['x'], graph.nodes[v]['y'])
            geom = LineString([p1, p2])

        if isinstance(geom, LineString) and geom.is_valid:
            edge_geoms[(u, v, key)] = geom
            edge_midpoints[(u, v, key)] = edge_midpoint(geom)
            edge_geom_list.append(geom)
            edge_keys_list.append((u, v, key))

    spatial_index = STRtree(edge_geom_list)
    neighbor_weights = defaultdict(list)

    # Step 2: buffer and intersect
    for (u, v, key), base_geom in edge_geoms.items():
        base_midpoint = edge_midpoints[(u, v, key)]
        buffer = base_geom.buffer(buffer_radius)
        candidates = spatial_index.query(buffer)

        for candidate_geom in candidates:
            if not isinstance(candidate_geom, LineString):
                continue

            try:
                idx = edge_geom_list.index(candidate_geom)
                cand_key = edge_keys_list[idx]
            except ValueError:
                continue

            if cand_key == (u, v, key):
                continue

            intersected = candidate_geom.intersection(buffer)
            if intersected.is_empty:
                continue

            if intersected.geom_type == 'GeometryCollection':
                parts = [g for g in intersected.geoms if isinstance(g, LineString)]
            elif intersected.geom_type == 'LineString':
                parts = [intersected]
            else:
                continue

            for part in parts:
                mid = edge_midpoint(part)
                d = base_midpoint.distance(mid)
                w = gaussian_weight(d, sigma)
                if w > 0:
                    neighbor_weights[(u, v, key)].append((cand_key, w))

    return neighbor_weights


def compute_path_metrics(graph, path):
    time = 0
    exposure = 0
    for u, v in zip(path[:-1], path[1:]):
        edge = graph[u][v][0]
        time += edge.get('walking_time', 1)
        exposure += edge.get('personal_exposure', 0)
    return time, exposure

def compute_path_hypervolume(time_diff, exposure_diff, local_ideal_time, local_ideal_exposure):
    return (local_ideal_time - time_diff) * (local_ideal_exposure - exposure_diff)

def get_pedestrian_path_with_weight(graph, origin, destination, weight):
    for u, v, key, data in graph.edges(keys=True, data=True):
        wt = data.get('walking_time_norm', 1)
        exp = data.get('personal_exposure_norm', 0)
        data[f'cost_{weight}'] = weight * wt + (1 - weight) * exp

    return nx.shortest_path(graph, origin, destination, weight=f'cost_{weight}')

def select_best_pedestrian_path_based_on_dynamic_exposure(graph, origin, destination, weights):
    paths = []
    metrics = []

    fastest_path = get_fastest_path(graph, origin, destination)
    fastest_time, fastest_exposure = compute_path_metrics(graph, fastest_path)
    
    for w in weights:
        # Compute combined cost
        for u, v, key, data in graph.edges(keys=True, data=True):
            wt = data.get('walking_time_norm', 1)
            exp = data.get('personal_exposure_norm', 0)
            data[f'cost_{w}'] = w * wt + (1 - w) * exp

        try:
            path = nx.shortest_path(graph, origin, destination, weight=f'cost_{w}')
            time, exposure = compute_path_metrics(graph, path)
            time_diff = ((time - fastest_time) / fastest_time) * 100 if fastest_time != 0 else 0
            exposure_diff = ((exposure - fastest_exposure) / fastest_exposure) * 100 if fastest_exposure != 0 else 0
            paths.append(path)
            metrics.append((w, path, time_diff, exposure_diff))
        except:
            continue

    if not metrics:
        return fastest_path  # fallback

    # Local ideal point (max diffs among alternatives)
    local_ideal_time = max(m[2] for m in metrics)
    local_ideal_exposure = max(m[3] for m in metrics)

    best_score = -np.inf
    best_path = None

    for w, path, t_diff, e_diff in metrics:
        score = compute_path_hypervolume(t_diff, e_diff, local_ideal_time, local_ideal_exposure)
        if score > best_score:
            best_score = score
            best_path = path

    return best_path

# def update_pedestrian_exposure(graph, total_emission_map):
#     for u, v, key, data in graph.edges(keys=True, data=True):
#         edge_key = tuple(sorted((u, v)))
#         emission = total_emission_map.get(edge_key, 0)
#         # if emission > 0:
#           # print(f"[DEBUG] Edge ({u}, {v}) has emission {emission:.2f}")
#         length = data.get('length', 1)
#         walking_time = data.get('walking_time', 1)
#         data['total_emission'] = emission
#         data['personal_exposure'] = (emission / length) * walking_time if length > 0 else 0
#         # (emission per meter * walking time)

#     # Normalize time & exposure again
#     walking_times = [d.get('walking_time', 1) for _, _, _, d in graph.edges(keys=True, data=True)]
#     exposures = [d.get('personal_exposure', 0) for _, _, _, d in graph.edges(keys=True, data=True)]
#     norm_time = normalize(walking_times)
#     norm_exp = normalize(exposures)

#     for i, (u, v, key, data) in enumerate(graph.edges(keys=True, data=True)):
#         data['walking_time_norm'] = norm_time[i]
#         data['personal_exposure_norm'] = norm_exp[i]


def update_pedestrian_exposure(graph, total_emission_map, neighbor_weights):
    for u, v, key, data in graph.edges(keys=True, data=True):
        edge_key = tuple(sorted((u, v)))
        own_emission = total_emission_map.get(edge_key, 0)

        neighbor_emission = sum(
            total_emission_map.get(tuple(sorted((nu, nv))), 0) * w
            for (nu, nv, _), w in neighbor_weights.get((u, v, key), [])
        )

        # consider dispersion from neighboring roads
        total = own_emission + neighbor_emission
        length = data.get('length', 1)
        walking_time = data.get('walking_time', 1)

        data['total_emission'] = total
        data['personal_exposure'] = (total / length) * walking_time if length > 0 else 0

    # Normalize time & exposure again
    walking_times = [d.get('walking_time', 1) for _, _, _, d in graph.edges(keys=True, data=True)]
    exposures = [d.get('personal_exposure', 0) for _, _, _, d in graph.edges(keys=True, data=True)]
    norm_time = normalize(walking_times)
    norm_exp = normalize(exposures)

    for i, (u, v, key, data) in enumerate(graph.edges(keys=True, data=True)):
        data['walking_time_norm'] = norm_time[i]
        data['personal_exposure_norm'] = norm_exp[i]
