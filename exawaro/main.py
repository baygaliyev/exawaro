from collections import defaultdict
import os

import networkx as nx
import numpy as np
import osmnx as ox
import pandas as pd
import pickle
import random
# print(ox.__version__)

from exawaro.cars import (compute_emission, compute_total_emission_map,
                          get_car_path_with_weight)
from exawaro.pedestrians import (build_neighbor_emission_weights,
                                 compute_path_metrics,
                                 get_pedestrian_path_with_weight,
                                 select_best_pedestrian_path_based_on_dynamic_exposure,
                                 update_pedestrian_exposure)
from exawaro.utils import normalize


REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
input_data_path = os.path.join(REPO_ROOT, 'data') + os.sep
output_dir = os.path.join(REPO_ROOT, 'output') + os.sep
os.makedirs(output_dir, exist_ok=True)

# Define Pisa center boundaries
# ymax, ymin, xmax, xmin = (43.7295, 43.7025, 10.4214, 10.3824) # Pisa 3x3
south, west, north, east = 43.7025, 10.3824, 43.7295, 10.4214

# weights for least-cost paths
weights = [round(0.8**x, 3) for x in range(0, 20, 2)]

# print("BBOX:", west, south, east, north)
ped_graph = ox.graph_from_bbox((west, south, east, north), network_type="walk", 
                               simplify=False, retain_all=True)

ped_graph = ox.convert.to_undirected(ped_graph)

car_graph = ox.graph_from_bbox((west, south, east, north), network_type="drive_service", 
                               simplify=False, retain_all=True)

car_graph = ox.add_edge_speeds(car_graph) # add speed limits
car_graph = ox.add_edge_travel_times(car_graph) # add travel times

# Consider neighboring edges in emission exposure
neighbor_weights = build_neighbor_emission_weights(ped_graph)

for u, v, k, data in ped_graph.edges(keys=True, data=True):
    if 'length' not in data or data['length'] is None:
        data['length'] = 1  # fallback value
    data['walking_time'] = data['length'] / 1.4  # assuming 1.4 m/s average walking speed

for u, v, k, data in car_graph.edges(keys=True, data=True):
    data['speed_limit_mps'] = data.get('speed_kph', 50) / 3.6
    data['driving_time'] = data.get('travel_time', data['length'] / data['speed_limit_mps'])

# Precompute emissions for car graph edges
precomputed_emission_map = {
    (u, v): compute_emission(
        data.get('speed_limit_mps', 13.9),  # or your column
        data.get('driving_time', data.get('length', 100) / data.get('speed_limit_mps', 13.9)))
    for u, v, data in car_graph.edges(data=True)}

# Load pedestrian & vehicular OD data
resident_trips = pd.read_csv(input_data_path + "noise_od_pairs_pisa.csv")
tourist_trips = pd.read_csv(input_data_path + "tourist_5000_od_pairs_pisa.csv")
worker_trips = pd.read_csv(input_data_path + "workers_students_od_pairs_pisa.csv")
car_trips = pd.read_csv(input_data_path + "cars_od_pairs_pisa_2526_routes.csv")

### --- Utility Functions --- ###

def compute_best_global_weight_from_metrics(metrics_df):
    weights = sorted(metrics_df['weight'].unique())
    pareto_results = []

    for w in weights:
        df_w = metrics_df[metrics_df['weight'] == w]

        time_diff = df_w['least_cost_time'] - df_w['fastest_time']
        exposure_diff = df_w['least_cost_exposure'] - df_w['fastest_exposure']

        time_diff_percentage = (time_diff / df_w['fastest_time']).replace([np.inf, -np.inf], 0).fillna(0) * 100
        exposure_diff_percentage = (exposure_diff / df_w['fastest_exposure']).replace([np.inf, -np.inf], 0).fillna(0) * 100

        pareto_results.append({
            'weight': w,
            'avg_time_diff_percentage': time_diff_percentage.mean(),
            'avg_exposure_diff_percentage': exposure_diff_percentage.mean()
        })

    pareto_df = pd.DataFrame(pareto_results)

    buffer_reference_time = pareto_df['avg_time_diff_percentage'].max() - pareto_df['avg_time_diff_percentage'].min()
    buffer_exposure = pareto_df['avg_exposure_diff_percentage'].max() - pareto_df['avg_exposure_diff_percentage'].min()

    ref_time = pareto_df['avg_time_diff_percentage'].max() + 0.1 * buffer_reference_time
    ref_exp = pareto_df['avg_exposure_diff_percentage'].max() + 0.1 * buffer_exposure

    # ref_time = pareto_df['avg_time_diff_percentage'].max()
    # ref_exp = pareto_df['avg_exposure_diff_percentage'].max()

    pareto_df['hypervolume'] = (
        (ref_time - pareto_df['avg_time_diff_percentage']) *
        (ref_exp - pareto_df['avg_exposure_diff_percentage'])
    )

    best_weight = pareto_df.loc[pareto_df['hypervolume'].idxmax(), 'weight']
    return best_weight, pareto_df

# storage for global weights per step
global_weights_log = []

def log_global_weights(step, ped_weight, car_weight):
    global_weights_log.append({
        'step': step,
        'global_ped_weight': ped_weight,
        'global_car_weight': car_weight
    })
    print(f"Step {step} | Pedestrian Global Weight: {ped_weight} | Car Global Weight: {car_weight}")

def get_fastest_path(graph, origin, destination):
    return nx.shortest_path(graph, origin, destination, weight='walking_time')

### --- Main Simulation Step --- ###

def run_simulation_step(t, ped_graph, car_graph, pedestrian_od_pairs, car_od_pairs,
                        precomputed_emission_map, weights, mode='local',
                        previous_total_emission_map=None, neighbor_weights=None):
    
    # Update exposure based on emissions from the previous step
    undirected_emission_map = defaultdict(float)
    for (u, v), e in previous_total_emission_map.items():
        key = tuple(sorted((u, v)))
        undirected_emission_map[key] += e

    update_pedestrian_exposure(ped_graph, undirected_emission_map, neighbor_weights)

    pedestrian_routes = []
    population_map = defaultdict(int)
    car_routes = []
    total_emission_map = defaultdict(float)

    if mode == 'global':
        # --- GLOBAL pedestrian routing ---
        ped_metrics = []
     
        for origin, destination in pedestrian_od_pairs:
            try:
                fastest_path = get_fastest_path(ped_graph, origin, destination)
                fastest_time, fastest_exposure = compute_path_metrics(ped_graph, fastest_path)

                for w in weights:
                    for u, v, key, data in ped_graph.edges(keys=True, data=True):
                        wt = data.get('walking_time_norm', 1)
                        exp = data.get('personal_exposure_norm', 0)
                        data[f'cost_{w}'] = w * wt + (1 - w) * exp

                    path = nx.shortest_path(ped_graph, origin, destination, weight=f'cost_{w}')
                    time, exposure = compute_path_metrics(ped_graph, path)

                    ped_metrics.append({
                        'origin': origin,
                        'destination': destination,
                        'weight': w,
                        'least_cost_time': time,
                        'least_cost_exposure': exposure,
                        'fastest_time': fastest_time,
                        'fastest_exposure': fastest_exposure
                    })
            except:
                continue

        ped_metrics_df = pd.DataFrame(ped_metrics)
        global_ped_weight, _ = compute_best_global_weight_from_metrics(ped_metrics_df)

        for origin, destination in pedestrian_od_pairs:
            try:
                if t == 0:
                    path = get_fastest_path(ped_graph, origin, destination)
                else:
                    path = get_pedestrian_path_with_weight(ped_graph, origin, destination, 
                                                        global_ped_weight)
                pedestrian_routes.append((origin, destination, path))
                for u, v in zip(path[:-1], path[1:]):
                    edge = tuple(sorted((u, v)))
                    population_map[edge] += 1
            except:
                continue

        # --- GLOBAL car routing ---
        car_metrics = []

        for origin, destination in car_od_pairs:
            try:
                fastest_path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                fastest_time = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(fastest_path[:-1], fastest_path[1:]))
                fastest_exposure = sum(car_graph[u][v][0].get('exposure', 0) for u, v in zip(fastest_path[:-1], fastest_path[1:]))

                for w in weights:
                    # Step 1: compute cost for all edges based on this weight
                    for u, v, key, data in car_graph.edges(keys=True, data=True):
                        time = data.get('driving_time', 1)
                        pop = population_map.get(tuple(sorted((u, v))), 0)
                        emis = precomputed_emission_map.get((u, v), 0)
                        exposure = emis * pop
                        data[f'cost_{w}'] = w * time + (1 - w) * exposure
                    
                    # Step 2: compute route and evaluate
                    path = nx.shortest_path(car_graph, origin, destination, weight=f'cost_{w}')
                    time = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(path[:-1], path[1:]))
                    exposure = sum(car_graph[u][v][0].get('exposure', 0) for u, v in zip(path[:-1], path[1:]))

                    car_metrics.append({
                        'origin': origin,
                        'destination': destination,
                        'weight': w,
                        'least_cost_time': time,
                        'least_cost_exposure': exposure,
                        'fastest_time': fastest_time,
                        'fastest_exposure': fastest_exposure
                    })
            except:
                continue

        car_metrics_df = pd.DataFrame(car_metrics)
        global_car_weight, _ = compute_best_global_weight_from_metrics(car_metrics_df)

        for origin, destination in car_od_pairs:
            try:
                if t == 0:
                    path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                else:
                    path = get_car_path_with_weight(car_graph, origin, destination, 
                                                    global_car_weight, population_map, 
                                                    precomputed_emission_map)
                car_routes.append((origin, destination, path))
            except:
                continue

        # Update total emissions based on car routes
        total_emission_map = compute_total_emission_map(car_routes, precomputed_emission_map)
        if t != 0:
            log_global_weights(t, global_ped_weight, global_car_weight)

    else:
        # --- LOCAL pedestrian routing ---
        for origin, destination in pedestrian_od_pairs:
            try:
                if t == 0:
                    path = get_fastest_path(ped_graph, origin, destination)
                else:
                    path = select_best_pedestrian_path_based_on_dynamic_exposure(ped_graph, origin, destination, weights)
                
                pedestrian_routes.append((origin, destination, path))
                for u, v in zip(path[:-1], path[1:]):
                    edge = tuple(sorted((u, v)))
                    population_map[edge] += 1
            except:
                continue

        # --- LOCAL car routing ---
        for origin, destination in car_od_pairs:
            if t == 0:
                try:
                    path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                    car_routes.append((origin, destination, path))
                except:
                    continue
            else:
                try:
                    # Step 0: Fastest route (baseline)
                    fastest_path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                    fastest_time = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(fastest_path[:-1], fastest_path[1:]))
                    fastest_emission = sum(precomputed_emission_map.get((u, v), 0) for u, v in zip(fastest_path[:-1], fastest_path[1:]))

                    path_metrics = []
                    for w in weights:
                        # Compute cost
                        for u, v, key, data in car_graph.edges(keys=True, data=True):
                            time = data.get('driving_time', 1)
                            pop = population_map.get(tuple(sorted((u, v))), 0)
                            emis = precomputed_emission_map.get((u, v), 0)
                            exposure = emis * pop
                            data[f'cost_{w}'] = w * time + (1 - w) * exposure

                        # Get path
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
                        continue

                    # Local ideal point
                    time_diffs = [m[2] for m in path_metrics]
                    emission_diffs = [m[3] for m in path_metrics]
                    buffer_time = max(time_diffs) - min(time_diffs)
                    buffer_emission = max(emission_diffs) - min(emission_diffs)
                    ideal_time = max(time_diffs) + 0.1 * buffer_time
                    ideal_emission = max(emission_diffs) + 0.1 * buffer_emission

                    best_score = -np.inf
                    best_path = None

                    for w, path, t_diff, e_diff in path_metrics:
                        score = (ideal_time - t_diff) * (ideal_emission - e_diff)
                        if score > best_score:
                            best_score = score
                            best_path = path

                    if best_path:
                        car_routes.append((origin, destination, best_path))
                except:
                    continue

        total_emission_map = compute_total_emission_map(car_routes, precomputed_emission_map)
        unique_routes = set(tuple(r[2]) for r in car_routes)
        print(f"[DEBUG] Step {t} | Unique car routes: {len(unique_routes)} of {len(car_routes)} total")

    if t == 0:
        for u, v, key, data in ped_graph.edges(keys=True, data=True):
            data['personal_exposure'] = 0

    # Update pedestrian exposure on undir roads based on dir car roads
    def make_undirected_emission_map(emission_map):
        undirected = defaultdict(float)
        for (u, v), value in emission_map.items():
            key = tuple(sorted((u, v)))
            undirected[key] += value
        return undirected

    undirected_emission_map = make_undirected_emission_map(previous_total_emission_map)
    update_pedestrian_exposure(ped_graph, undirected_emission_map, neighbor_weights)

    # DEBUG: check exposure distribution
    exposures = [data['personal_exposure'] for _, _, _, data in ped_graph.edges(keys=True, data=True)]
    print(f"[DEBUG] Step {t} | Exposure range: min={min(exposures):.6f}, max={max(exposures):.6f}")

    # Normalize walking time and personal exposure again
    walking_times = [d.get('walking_time', 1) for _, _, _, d in ped_graph.edges(keys=True, data=True)]
    exposures = [d.get('personal_exposure', 0) for _, _, _, d in ped_graph.edges(keys=True, data=True)]

    norm_time = normalize(walking_times)
    norm_exposure = normalize(exposures)

    for i, (u, v, key, data) in enumerate(ped_graph.edges(keys=True, data=True)):
        data['walking_time_norm'] = norm_time[i]
        data['personal_exposure_norm'] = norm_exposure[i]

    return ped_graph, car_graph, pedestrian_routes, car_routes, population_map, total_emission_map



### --- Full Simulation Loop --- ###

def run_simulation(T, ped_graph, car_graph, pedestrian_od_pairs, car_od_pairs,
                   precomputed_emission_map, weights, mode='local', neighbor_weights=None):

    all_results = {}
    step_summary_log = []

    previous_total_emission_map = defaultdict(float)  # Step -1: zero emission

    for t in range(T):
        ped_graph, car_graph, pedestrian_routes, car_routes, population_map, total_emission_map = \
        run_simulation_step(t, ped_graph, car_graph, pedestrian_od_pairs, 
                            car_od_pairs, precomputed_emission_map, weights, mode=mode,
                            previous_total_emission_map=previous_total_emission_map,
                            neighbor_weights=neighbor_weights)
        
        # Save results for this time step
        all_results[t] = {
        'ped_graph': ped_graph.copy(),  # optionally use deepcopy if needed
        'car_graph': car_graph.copy(),
        'pedestrian_routes': pedestrian_routes,
        'car_routes': car_routes,
        'population_map': population_map.copy(),
        'total_emission_map': total_emission_map.copy()}

        previous_total_emission_map = total_emission_map.copy()
        
        # Precompute and store fastest path metrics for later comparisons
        if t == 0:
            # Clear personal exposure before computing fastest route exposure
            # for u, v, key, data in ped_graph.edges(keys=True, data=True):
            #     data['personal_exposure'] = 0

            # fastest_exposure_per_od = {}
            fastest_walk_time_per_od = {}

            for origin, destination, _ in pedestrian_routes:
                try:
                    path = get_fastest_path(ped_graph, origin, destination)
                    time, exposure = compute_path_metrics(ped_graph, path)
                    fastest_walk_time_per_od[(origin, destination)] = time
                    # fastest_exposure_per_od[(origin, destination)] = exposure
                except:
                    continue

            all_results[t]['fastest_walk_time_per_od'] = fastest_walk_time_per_od
            # all_results[t]['fastest_exposure_per_od'] = fastest_exposure_per_od



        # Summary for this time step
        total_exposure = sum(data.get('personal_exposure', 0) for _, _, _, data in ped_graph.edges(keys=True, data=True))
        # total_pedestrians = sum(population_map.values())
        # mean_exposure_per_person = total_exposure / total_pedestrians if total_pedestrians else 0

        total_emission = sum(total_emission_map.values())
        print(f"[DEBUG] Step {t} | Total emissions: {total_emission:.2f}")

        # total_cars = len(car_routes)
        # mean_emission_per_car = total_emission / total_cars if total_cars else 0

        # Use precomputed fastest path metrics from step 0
        # fastest_exposure_per_od = all_results[0].get('fastest_exposure_per_od', {})
        fastest_walk_time_per_od = all_results[0].get('fastest_walk_time_per_od', {})

        total_fastest_exposure = 0
        total_fastest_walk_time = 0

        for origin, destination, _ in pedestrian_routes:
            try:
                # Freshly recompute exposure from current conditions
                path_fastest = get_fastest_path(ped_graph, origin, destination)
                time, exposure = compute_path_metrics(ped_graph, path_fastest)
                total_fastest_exposure += exposure

                # Still use stored walk time from step 0
                total_fastest_walk_time += fastest_walk_time_per_od[(origin, destination)]
            except:
                continue

        # Compute total fastest-path car emission
        total_fastest_emission = 0
        for origin, destination, _ in car_routes:
            try:
                path = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                for u, v in zip(path[:-1], path[1:]):
                    total_fastest_emission += precomputed_emission_map.get((u, v), 0)
            except:
                continue

        # Percent difference in exposure and emissions compared to fastest
        exposure_pct_diff = ((total_exposure - total_fastest_exposure) / total_fastest_exposure * 100) if total_fastest_exposure else 0
        emission_pct_diff = ((total_emission - total_fastest_emission) / total_fastest_emission * 100) if total_fastest_emission else 0

        print(f"→ Step {t} | Exposure Δ from fastest: {exposure_pct_diff:.2f}% | Emission Δ from fastest: {emission_pct_diff:.2f}%")

        # Compute pedestrian time percentage difference
        total_walk_time = 0
        total_fastest_walk_time = 0
        for origin, destination, path in pedestrian_routes:
            time_actual, _ = compute_path_metrics(ped_graph, path)
            total_walk_time += time_actual
            try:
                path_fastest = get_fastest_path(ped_graph, origin, destination)
                time_fastest, _ = compute_path_metrics(ped_graph, path_fastest)
                total_fastest_walk_time += time_fastest
            except:
                continue

        # Compute car time percentage difference
        total_drive_time = 0
        total_fastest_drive_time = 0
        for origin, destination, path in car_routes:
            time_actual = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(path[:-1], path[1:]))
            total_drive_time += time_actual
            try:
                path_fastest = nx.shortest_path(car_graph, origin, destination, weight='driving_time')
                time_fastest = sum(car_graph[u][v][0].get('driving_time', 1) for u, v in zip(path_fastest[:-1], path_fastest[1:]))
                total_fastest_drive_time += time_fastest
            except:
                continue

        walk_time_pct_diff = ((total_walk_time - total_fastest_walk_time) / total_fastest_walk_time * 100) if total_fastest_walk_time else 0
        drive_time_pct_diff = ((total_drive_time - total_fastest_drive_time) / total_fastest_drive_time * 100) if total_fastest_drive_time else 0

        step_summary_log.append({
            "step": t,
            "total_exposure": total_exposure,
            "fastest_exposure": total_fastest_exposure,
            "exposure_pct_diff": exposure_pct_diff,

            "total_emission": total_emission,
            "fastest_emission": total_fastest_emission,
            "emission_pct_diff": emission_pct_diff,

            "total_walk_time": total_walk_time,
            "fastest_walk_time": total_fastest_walk_time,
            "walk_time_pct_diff": walk_time_pct_diff,

            "total_drive_time": total_drive_time,
            "fastest_drive_time": total_fastest_drive_time,
            "drive_time_pct_diff": drive_time_pct_diff,
        })

        print(f"→ Step {t} | Walk time Δ from fastest: {walk_time_pct_diff:.2f}% | Drive time Δ: {drive_time_pct_diff:.2f}%")

    pd.DataFrame(step_summary_log).to_csv(output_dir + f"{mode}_step_summary.csv", index=False)

    return all_results

# ========= PARAMETERS =====================

# obtain OD nodes
resident_od_pairs = list(zip(resident_trips['origin_node'], resident_trips['destination_node']))
worker_od_pairs = list(zip(worker_trips['origin_node'], worker_trips['destination_node']))
tourist_od_pairs = list(zip(tourist_trips['origin_node'], tourist_trips['destination_node']))
car_od_pairs = list(zip(car_trips['origin_node'], car_trips['destination_node']))

# (optional) sample a subset of OD pairs
def _sample_up_to(pairs, n):
    return random.sample(pairs, min(n, len(pairs)))

resident_od_pairs = _sample_up_to(resident_od_pairs, 1500)
worker_od_pairs = _sample_up_to(worker_od_pairs, 1500)
tourist_od_pairs = _sample_up_to(tourist_od_pairs, 1000)
pedestrian_od_pairs = resident_od_pairs + worker_od_pairs + tourist_od_pairs
car_od_pairs = _sample_up_to(car_od_pairs, 1000)

modes_steps = {
    'global': 7,
    'local': 5
}

for mode, num_steps in modes_steps.items():
    print()
    print(f"Running simulation in {mode} mode for {num_steps} steps...")
    print()
    # Run simulation
    results = run_simulation(num_steps, ped_graph, car_graph, pedestrian_od_pairs, 
                            car_od_pairs, precomputed_emission_map, weights, 
                            mode=mode, neighbor_weights=neighbor_weights)
    # Save results
    with open(output_dir + f'{mode}_simulation_results_{num_steps}_steps.pkl', 'wb') as f:
        pickle.dump(results, f)

    for t, step in results.items():
        # Save emission map
        pd.DataFrame([
            {'u': u, 'v': v, 'emission': e}
            for (u, v), e in step['total_emission_map'].items()
        ]).to_csv(output_dir + f"emission_map_{mode}_step_{t}.csv", index=False)

        # Save population map
        pd.DataFrame([
            {'u': u, 'v': v, 'population': p}
            for (u, v), p in step['population_map'].items()
        ]).to_csv(output_dir + f"population_map_{mode}_step_{t}.csv", index=False)

        print(f"Step {t} | {mode} | pedestrian_routes: {len(step['pedestrian_routes'])}, car_routes: {len(step['car_routes'])}")

        # Save pedestrian/car routes
        pd.DataFrame(step['pedestrian_routes'], columns=['origin', 'destination', 'path']).to_csv(
            output_dir + f"pedestrian_routes_{mode}_step_{t}.csv", index=False)
        pd.DataFrame(step['car_routes'], columns=['origin', 'destination', 'path']).to_csv(
            output_dir + f"car_routes_{mode}_step_{t}.csv", index=False)
