"""Generate origin-destination CSVs from restricted GPS trajectories.

Run with ``python -m exawaro.vehicular_od_generator_pisa``; the script reads data at import time.
"""

import os

import geopandas as gpd
import osmnx as ox
import pandas as pd
import random
from pyproj import Transformer
from shapely.geometry import Point, box
import numpy as np

# Paths and coordinate reference systems.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PATH_TO_INPUT_FILE = os.path.join(REPO_ROOT, 'data', 'trajectories') + os.sep
INPUT_FILE = 'italy_trajectories_week_22.csv'
PATH_TO_OUTPUT = os.path.join(REPO_ROOT, 'data') + os.sep
CRS_LATLON = "EPSG:4326"
CRS_UTM = "EPSG:32632"

# Bounding box in WGS84 (approx. 3x3 km)
xmin, ymin, xmax, ymax = 10.3824, 43.7025, 10.4214, 43.7295
bbox_geom = box(xmin, ymin, xmax, ymax)

# Build a 200 m grid over the bounding box.
# Transform the bounding box to UTM metres.
transformer = Transformer.from_crs(CRS_LATLON, CRS_UTM, always_xy=True)
xmin_m, ymin_m = transformer.transform(xmin, ymin)
xmax_m, ymax_m = transformer.transform(xmax, ymax)

# Create the 200 m by 200 m cells.
cell_size_m = 200
x_vals = np.arange(xmin_m, xmax_m, cell_size_m)
y_vals = np.arange(ymin_m, ymax_m, cell_size_m)
grid_cells_m = [box(x, y, x + cell_size_m, y + cell_size_m) for x in x_vals for y in y_vals]

grid_m = gpd.GeoDataFrame(geometry=grid_cells_m, crs=CRS_UTM)
grid = grid_m.to_crs(CRS_LATLON)  # back to WGS84
grid["tile_ID"] = range(len(grid))

# Keep the first and last point of each trajectory inside the box.
df = pd.read_csv(PATH_TO_INPUT_FILE + INPUT_FILE)
df["datetime"] = pd.to_datetime(df["datetime"])
df["geometry"] = df.apply(lambda row: Point(row["lng"], row["lat"]), axis=1)
gdf = gpd.GeoDataFrame(df, geometry="geometry", crs=CRS_LATLON)

# Sort each trajectory by time.
gdf = gdf.sort_values(["tid", "datetime"])

# Mark points inside the bounding box.
gdf["in_bbox"] = gdf.geometry.within(bbox_geom)

# Use those points as trip endpoints.
od_filtered = gdf[gdf["in_bbox"]].copy()
origins = od_filtered.groupby("tid").first().reset_index()
destinations = od_filtered.groupby("tid").last().reset_index()

# Combine endpoints into OD pairs.
od_df = origins[["tid", "geometry"]].merge(
    destinations[["tid", "geometry"]],
    on="tid", suffixes=("_origin", "_destination")
)

# Assign endpoints to grid tiles.
gdf_orig = gpd.GeoDataFrame(od_df, geometry="geometry_origin", crs=CRS_LATLON)
gdf_dest = gpd.GeoDataFrame(od_df, geometry="geometry_destination", crs=CRS_LATLON)

origin_cells = gpd.sjoin(gdf_orig, grid[["tile_ID", "geometry"]], how="left", predicate="within")
destination_cells = gpd.sjoin(gdf_dest, grid[["tile_ID", "geometry"]], how="left", predicate="within")

od_df["origin_tile"] = origin_cells["tile_ID"].values
od_df["destination_tile"] = destination_cells["tile_ID"].values

# Drop pairs with no tile assignment.
od_df = od_df.dropna(subset=["origin_tile", "destination_tile"])

# Count trips per tile pair.
demand_matrix = od_df.groupby(["origin_tile", "destination_tile"]).size().reset_index(name="trip_count")

# Attach OSM nodes to the grid tiles.
graph = ox.graph_from_bbox(north=ymax, south=ymin, east=xmax, west=xmin, network_type="drive_service")
nodes, _ = ox.graph_to_gdfs(graph, nodes=True, edges=True)

# Project nodes and grid to UTM for the spatial join.
grid_proj = grid.to_crs(CRS_UTM)
nodes_proj = nodes.to_crs(CRS_UTM)

# Assign each node to its tile.
node_tiles = gpd.sjoin(nodes_proj, grid_proj[["tile_ID", "geometry"]], how="left", predicate="within")

# Sample routes between tile pairs.
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

# Write the OD table and print a preview.
routes_df = pd.DataFrame(routes, columns=["origin_node", "destination_node", "origin_tile", "destination_tile"])

# Transformer from UTM zone 32N back to WGS84.
transformer_to_latlon = Transformer.from_crs(CRS_UTM, CRS_LATLON, always_xy=True)

# Get UTM coordinates for the sampled nodes.
routes_df["origin"] = routes_df["origin_node"].apply(lambda n: nodes_proj.loc[n].geometry.coords[0])
routes_df["destination"] = routes_df["destination_node"].apply(lambda n: nodes_proj.loc[n].geometry.coords[0])

# Convert UTM coordinates back to latitude and longitude.
routes_df[["origin_lon", "origin_lat"]] = routes_df["origin"].apply(lambda xy: pd.Series(transformer_to_latlon.transform(*xy)))
routes_df[["dest_lon", "dest_lat"]] = routes_df["destination"].apply(lambda xy: pd.Series(transformer_to_latlon.transform(*xy)))

# Keep the columns used by the simulation.
routes_df = routes_df[[
    "origin", "destination", "origin_lon", "origin_lat", "dest_lon", "dest_lat",
    "origin_tile", "destination_tile", "origin_node", "destination_node",
]]

# Write the OD table and print a preview.
output_name = "cars_od_pairs_pisa.csv"
os.makedirs(PATH_TO_OUTPUT, exist_ok=True)
routes_df.to_csv(PATH_TO_OUTPUT + output_name, index=False)
print(f"Routes saved to {PATH_TO_OUTPUT + output_name}")
print(routes_df.head())
