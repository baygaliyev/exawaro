import networkx as nx
import numpy as np

def compute_emission(speed, driving_time):
    E_1 = (0.553 + 0.161 * speed - 0.00289 * (speed ** 2)) * driving_time
    return max(0, E_1)

def compute_total_emission_map(car_routes, precomputed_emission_map):
    """
    Computes the total emissions per edge for a simulation step based on car routes.

    Parameters:
    - car_routes: list of (origin, destination, path), where path is a list of nodes
    - precomputed_emission_map: dict[(u, v)] -> emission per traversal

    Returns:
    - total_emission_map: dict[(u, v)] -> total emissions accumulated over all car routes
    """
    from collections import defaultdict
    total_emission_map = defaultdict(float)

    for _, _, path in car_routes:
        for u, v in zip(path[:-1], path[1:]):
            edge = tuple(sorted((u, v)))
            total_emission_map[edge] += precomputed_emission_map.get((u, v), 0.0)

    return total_emission_map


def get_car_path_with_weight(graph, origin, destination, weight, population_map, precomputed_emission_map):
    for u, v, key, data in graph.edges(keys=True, data=True):
        time = data.get('driving_time', 1)
        pop = population_map.get(tuple(sorted((u, v))), 0)
        emis = precomputed_emission_map.get((u, v), 0)
        exposure = emis * pop
        data[f'cost_{weight}'] = weight * time + (1 - weight) * exposure

    return nx.shortest_path(graph, origin, destination, weight=f'cost_{weight}')

def select_best_car_path(car_graph, origin, destination, weights, population_map, precomputed_emission_map):
    # Step 1: compute fastest route as baseline
    try:
        fastest_path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
    except:
        return None

    fastest_time = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(fastest_path[:-1], fastest_path[1:]))
    fastest_emission = sum(precomputed_emission_map.get((u, v), 0) for u, v in zip(fastest_path[:-1], fastest_path[1:]))

    path_metrics = []

    for w in weights:
        for u, v, key, data in car_graph.edges(keys=True, data=True):
            time = data.get('driving_time', 1)
            pop = population_map.get(tuple(sorted((u, v))), 0)
            emis = precomputed_emission_map.get((u, v), 0)
            exposure = emis * pop
            data[f'cost_{w}'] = w * time + (1 - w) * exposure

        try:
            path = nx.shortest_path(car_graph, origin, destination, weight=f'cost_{w}')
            time = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(path[:-1], path[1:]))
            emission = sum(precomputed_emission_map.get((u, v), 0) for u, v in zip(path[:-1], path[1:]))
            time_diff = ((time - fastest_time) / fastest_time) * 100 if fastest_time else 0
            emission_diff = ((emission - fastest_emission) / fastest_emission) * 100 if fastest_emission else 0
            path_metrics.append((w, path, time_diff, emission_diff))
        except:
            continue

    if not path_metrics:
        return fastest_path

    time_diffs = [m[2] for m in path_metrics]
    emission_diffs = [m[3] for m in path_metrics]
    
    buffer_time = max(time_diffs) - min(time_diffs)
    buffer_emission = max(emission_diffs) - min(emission_diffs)
    
    ideal_time = max(time_diffs) + 0.1 * buffer_time
    ideal_emission = max(emission_diffs) + 0.1 * buffer_emission

    best_score = -np.inf
    best_path = fastest_path

    for w, path, t_diff, e_diff in path_metrics:
        score = (ideal_time - t_diff) * (ideal_emission - e_diff)
        if score > best_score:
            best_score = score
            best_path = path

    return best_path
