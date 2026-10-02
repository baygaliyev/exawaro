import os

import geopandas as gpd
import osmnx as ox
import pandas as pd
import random
from pyproj import Transformer
from shapely.geometry import Point, box
import numpy as np

# ------------------ SETTINGS ------------------
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_INPUT_FILE = os.path.join(REPO_ROOT, 'data', 'trajectories') + os.sep
INPUT_FILE = 'italy_trajectories_week_22.csv'
PATH_TO_OUTPUT = os.path.join(REPO_ROOT, 'data') + os.sep
CRS_LATLON = "EPSG:4326"
CRS_UTM = "EPSG:32632"

# Bounding box in WGS84 (approx. 3x3 km)
xmin, ymin, xmax, ymax = 10.3824, 43.7025, 10.4214, 43.7295
bbox_geom = box(xmin, ymin, xmax, ymax)

# ------------------ GRID GENERATION ------------------
# Transform to meters (UTM)
transformer = Transformer.from_crs(CRS_LATLON, CRS_UTM, always_xy=True)
xmin_m, ymin_m = transformer.transform(xmin, ymin)
xmax_m, ymax_m = transformer.transform(xmax, ymax)

# Generate 200m x 200m grid in meters
cell_size_m = 200
x_vals = np.arange(xmin_m, xmax_m, cell_size_m)
y_vals = np.arange(ymin_m, ymax_m, cell_size_m)
grid_cells_m = [box(x, y, x + cell_size_m, y + cell_size_m) for x in x_vals for y in y_vals]

grid_m = gpd.GeoDataFrame(geometry=grid_cells_m, crs=CRS_UTM)
grid = grid_m.to_crs(CRS_LATLON)  # back to WGS84
grid["tile_ID"] = range(len(grid))

# ------------------ LOAD AND FILTER TRAJECTORIES ------------------
df = pd.read_csv(PATH_TO_INPUT_FILE + INPUT_FILE)
df["datetime"] = pd.to_datetime(df["datetime"])
df["geometry"] = df.apply(lambda row: Point(row["lng"], row["lat"]), axis=1)
gdf = gpd.GeoDataFrame(df, geometry="geometry", crs=CRS_LATLON)

# Sort by tid and datetime
gdf = gdf.sort_values(["tid", "datetime"])

# Identify points inside bounding box
gdf["in_bbox"] = gdf.geometry.within(bbox_geom)

# Filter to first and last point IN the box per trajectory
od_filtered = gdf[gdf["in_bbox"]].copy()
origins = od_filtered.groupby("tid").first().reset_index()
destinations = od_filtered.groupby("tid").last().reset_index()

# Create OD pairs
od_df = origins[["tid", "geometry"]].merge(
    destinations[["tid", "geometry"]],
    on="tid", suffixes=("_origin", "_destination")
)

# ------------------ TILE ASSIGNMENT ------------------
gdf_orig = gpd.GeoDataFrame(od_df, geometry="geometry_origin", crs=CRS_LATLON)
gdf_dest = gpd.GeoDataFrame(od_df, geometry="geometry_destination", crs=CRS_LATLON)

origin_cells = gpd.sjoin(gdf_orig, grid[["tile_ID", "geometry"]], how="left", predicate="within")
destination_cells = gpd.sjoin(gdf_dest, grid[["tile_ID", "geometry"]], how="left", predicate="within")

od_df["origin_tile"] = origin_cells["tile_ID"].values
od_df["destination_tile"] = destination_cells["tile_ID"].values

# Drop OD pairs outside the box
od_df = od_df.dropna(subset=["origin_tile", "destination_tile"])

# ------------------ BUILD DEMAND MATRIX ------------------
demand_matrix = od_df.groupby(["origin_tile", "destination_tile"]).size().reset_index(name="trip_count")

# ------------------ GRAPH AND NODE ASSIGNMENT ------------------
graph = ox.graph_from_bbox(north=ymax, south=ymin, east=xmax, west=xmin, network_type="drive_service")
nodes, edges = ox.graph_to_gdfs(graph, nodes=True, edges=True)

# Project nodes and grid to UTM
grid_proj = grid.to_crs(CRS_UTM)
nodes_proj = nodes.to_crs(CRS_UTM)

# Assign each node to a tile
node_tiles = gpd.sjoin(nodes_proj, grid_proj[["tile_ID", "geometry"]], how="left", predicate="within")

# ------------------ GENERATE OD ROUTES ------------------
routes = []
for _, row in demand_matrix.iterrows():
    origin_tile = row["origin_tile"]
    destination_tile = row["destination_tile"]
    num_trips = int(row["trip_count"])

    origin_nodes = node_tiles[node_tiles["tile_ID"] == origin_tile].index.tolist()
    dest_nodes = node_tiles[node_tiles["tile_ID"] == destination_tile].index.tolist()

    if not origin_nodes or not dest_nodes:
        continue

    for _ in range(num_trips):
        origin_node = random.choice(origin_nodes)
        destination_node = random.choice(dest_nodes)
        routes.append((origin_node, destination_node, origin_tile, destination_tile))

# ------------------ FINAL OUTPUT ------------------
routes_df = pd.DataFrame(routes, columns=["origin_node", "destination_node", "origin_tile", "destination_tile"])

# Create transformer: WGS84 ← UTM (zone 32N, Pisa area)
transformer_to_latlon = Transformer.from_crs(CRS_UTM, CRS_LATLON, always_xy=True)

# Get UTM coordinates of origin and destination nodes
routes_df["origin"] = routes_df["origin_node"].apply(lambda n: nodes_proj.loc[n].geometry.coords[0])
routes_df["destination"] = routes_df["destination_node"].apply(lambda n: nodes_proj.loc[n].geometry.coords[0])

# Convert UTM to lat/lon
routes_df[["origin_lon", "origin_lat"]] = routes_df["origin"].apply(lambda xy: pd.Series(transformer_to_latlon.transform(*xy)))
routes_df[["dest_lon", "dest_lat"]] = routes_df["destination"].apply(lambda xy: pd.Series(transformer_to_latlon.transform(*xy)))

# Reorder columns
routes_df = routes_df[[  
    "origin", "destination", "origin_lon", "origin_lat", "dest_lon", "dest_lat",  
    "origin_tile", "destination_tile", "origin_node", "destination_node"  
]]

# Save and show output
output_name = "cars_od_pairs_pisa.csv"
os.makedirs(PATH_TO_OUTPUT, exist_ok=True)
routes_df.to_csv(PATH_TO_OUTPUT + output_name, index=False)
print(f"Routes saved to {PATH_TO_OUTPUT + output_name}")
print(routes_df.head())
