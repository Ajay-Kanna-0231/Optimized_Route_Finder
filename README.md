
# Optimized Route Finder

A project to determine the fastest route between two geographical points (In the project chennai city is used) using OpenStreetMap data, considering speed limits of each road and custom speeds and what type of road it is.



## Features

- Calculates shortest travel time using Dijkstra's algorithm.
- Considers road types and their default speed limits.
- Allows custom speed input.
- Outputs travel time, distance traveled and visualizes the path taken on the map.



## Installation/ Setup:

- Clone the repository:

```bash
  git clone https://github.com/yourusername/Optimized_Route_Finder.git

```
- Navigate to the project directory:
```bash
cd Optimized_Route_Finder
 ```
    
- Create and activate a virtual environment:
```bash
python -m venv venv
venv\Scripts\activate # On macOS/Linux use `source venv/bin/activate`

```
- Install the required packages:
```bash
pip install -r requirements.txt
```
## How to use?
### Retrieve Edges (Roads) and Nodes (Junctions):
- Run the "retrieving_edges_nodes.py" file.

- Update the GPS coordinates (north, south, east, west) to define the city borders using Google Maps or OpenStreetMap.

- The retrieved graph data is saved in path_to_graph.pkl.

### Find Fastest Route:
- Run Find_Fastest_Route.py.
- Update the GPS coordinates of the start and end positions:
```python
start_lat, start_lon = 12.982987930788196, 80.2199129380192
end_lat, end_lon = 13.073064487133829, 80.257024490253
```
- Feel free to experiment with different locations and city :)


## Functions
- get_edge_time(properties, given_speed): Calculates the travel time for an edge.
- is_edge_allowed(properties, current_node, neighbor, given_speed): Checks if an edge is allowed based on oneway and access restrictions.
- dijkstra(graph, start, end, given_speed): Implements Dijkstra's algorithm to find the shortest travel time path.
## Contributing

Contributions are welcome! Please open an issue or submit a pull request.



## Authors

- [@Ajay Kanna](https://github.com/Ajay-Kanna-0231)


