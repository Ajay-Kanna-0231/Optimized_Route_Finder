import osmnx as ox
import networkx as nx
import pickle

# Define the place name
north, south, east, west = 13.2320, 12.9750, 80.2871, 80.1695

# Create graph from place
try:
    #print(f"Attempting to create graph from place '{place_name}'...")
    road = ox.graph_from_bbox(north, south, east, west, network_type='drive')
    print("Graph created successfully.")

    # Project the graph
    G = ox.project_graph(road)
    print("Graph projected successfully.")

    # Get nodes and edges
    nodes, edges = ox.graph_to_gdfs(G)
    print("Nodes and edges extracted successfully.")

    # Print some information about nodes and edges
    print(f"Number of nodes: {len(nodes)}")
    print(f"Number of edges: {len(edges)}")

    print("Node columns:", nodes.columns)
    print("Edge columns:", edges.columns)

    print("Node data summary:")
    print(nodes.describe(include='all'))  # Include 'all' to see statistics for non-numeric columns too

    print("Edge data summary:")
    print(edges.describe(include='all'))

    print("Detailed info on nodes:")
    print(nodes.info())

    print("Detailed info on edges:")
    print(edges.info())
    
except Exception as e:
    print(f"An error occurred: {e}")
finally:
    # Save the graph to a pickle file
    with open('path_to_graph.pkl', 'wb') as f:
        pickle.dump(G, f)
    print("Graph saved to 'path_to_graph.pkl'.")
