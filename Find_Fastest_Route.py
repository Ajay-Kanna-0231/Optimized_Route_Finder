import heapq
import pickle
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import geopandas as gpd
from shapely.geometry import Point
from collections import defaultdict


with open('path_to_graph.pkl', 'rb') as f:
    G = pickle.load(f)

def get_edge_time(properties, given_speed):

    length = properties[0]['length']
    maxspeed = properties[0].get('maxspeed', None)
    highway_type = properties[0].get('highway', 'road')
    if isinstance(highway_type, list):   # Handle highway_type if it's a list
            highway_type = highway_type[0]

    default_speeds = {'motorway': 100, 'motorway_link': 60, 'trunk': 80, 'trunk_link': 50, 
    'primary': 60, 'primary_link': 40, 'secondary': 50, 'secondary_link': 30, 
    'tertiary': 40, 'tertiary_link': 30, 'residential': 30, 'service': 20, 
    'unclassified': 30, 'road': 30, 'living_street': 10, 'pedestrian': 5, 
    'footway': 5, 'cycleway': 15, 'path': 10, 'track': 10, 'construction': 5, 
    'bus_guideway': 40}

    if maxspeed:
        try:
            if isinstance(maxspeed, list):   # Handle maxspeed if it's a list
                maxspeed = maxspeed[0]

            speed_m_per_min = float(maxspeed) * 1000 / 60
            if speed_m_per_min > given_speed:
                return length / given_speed, highway_type    # time in minutes
            else:
                return length / speed_m_per_min, highway_type
            
        except (ValueError, TypeError):
            pass

    default_speed = default_speeds.get(highway_type, 30)
    speed_m_per_min = default_speed * 1000 / 60
    if speed_m_per_min > given_speed:
        return length / given_speed, highway_type    # time in minutes
    else:
        return length / speed_m_per_min, highway_type


def is_edge_allowed(properties, current_node, neighbor, given_speed):
    if properties[0].get('oneway', False) and properties[0].get('reversed', False):
        if current_node > neighbor:
            return False
    
    if properties[0].get('access', 'yes') not in ['yes', 'permissive']:
        return False

    return True


def dijkstra(graph, start, end, given_speed):
    times = {node: float('infinity') for node in graph.nodes}
    previous_nodes = {node: None for node in graph.nodes}
    path_distances = {node: 0 for node in graph.nodes}
    road_type_count = defaultdict(int)
    
    times[start] = 0
    priority_queue = [(0, start)]
    
    while priority_queue:
        current_time, current_node = heapq.heappop(priority_queue)
        if current_time > times[current_node]:
            continue

        if current_node == end:
            break

        for neighbor, properties in graph.adj[current_node].items():
            if not is_edge_allowed(properties, current_node, neighbor, given_speed):
                continue

            edge_time, highway_type = get_edge_time(properties, given_speed)
            road_type_count[highway_type] += 1  # Count the road type
            edge_distance = properties[0]['length']
            time = current_time + edge_time
            distance = path_distances[current_node] + edge_distance

            if time < times[neighbor]:
                times[neighbor] = time
                path_distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (time, neighbor))

    path = []
    node = end
    while previous_nodes[node] is not None:
        path.insert(0, node)
        node = previous_nodes[node]
    if path:
        path.insert(0, node)
    total_distance = path_distances[end]
    
    return path, times[end], total_distance, road_type_count


# Assuming 'G' is your graph already loaded
start_lat, start_lon = 12.982987930788196, 80.2199129380192
end_lat, end_lon = 13.073064487133829, 80.257024490253

# Create a GeoDataFrame with the coordinates
gdf = gpd.GeoDataFrame(geometry=[Point(start_lon, start_lat), Point(end_lon, end_lat)], crs='EPSG:4326')

# Project this GeoDataFrame to the same CRS as the graph
gdf_projected = gdf.to_crs(G.graph['crs'])

# Extract the projected points
point_start = gdf_projected.geometry.iloc[0]
point_end = gdf_projected.geometry.iloc[1]

# Find the nearest nodes to these projected points
start_node = ox.distance.nearest_nodes(G, X=point_start.x, Y=point_start.y)
end_node = ox.distance.nearest_nodes(G, X=point_end.x, Y=point_end.y)


print("Start Node:", start_node)
print("End Node:", end_node)

print("Number of nodes in chennai:", len(G.nodes))
print("Number of edges in chennai:", len(G.edges))
print("Graph CRS:", G.graph['crs'])

given_speed = 50 * 1000/60
# Run Dijkstra's algorithm
path, total_time, total_distance, road_types = dijkstra(G, start_node, end_node, given_speed)

# Print results
print("Path:", path)
print(len(path))
print("Total Time Taken:", total_time)
print("Total Distance Traveled:", total_distance)
print(road_types)
# Plot the path on the graph
fig, ax = ox.plot_graph_route(G, path, node_size=0)
plt.show()