"""
Example: Using the MapMatcher for automatic splitting and time interpolation

This example shows how to use the high-level MapMatcher class that handles
matching failures automatically and interpolates timestamps.
"""
import sys
sys.path.insert(0, "build/python/pybind11/Release")

from pathlib import Path
import fmm

# Create a simple road network
network = fmm.Network()

# Create a longer road for realistic matching
line = fmm.LineString()
for i in range(0, 101, 10):
    line.add_point(float(i), 0.0)
network.add_edge(edge_id=1, source=1, target=2, geom=line)

# Add a perpendicular road
line2 = fmm.LineString()
line2.add_point(50.0, 0.0)
line2.add_point(50.0, 50.0)
network.add_edge(edge_id=2, source=2, target=3, geom=line2)

network.build_rtree_index()

print("Network created:")
print(f"  Edges: {network.get_edge_count()}")
print(f"  Nodes: {network.get_node_count()}")

# Create matcher (will generate UBODT if it doesn't exist)
cache_dir = Path("./cache")
cache_dir.mkdir(exist_ok=True)

print("\nInitializing MapMatcher...")
matcher = fmm.MapMatcher(
    network=network,
    ubodt_max_djikstra_distance=5000.0,
    cache_dir=cache_dir
)

print("\nMapMatcher initialized successfully!")
print("\nNote: For trajectory matching, you would use matcher.match()")
print("See the MapMatcher class for available methods.")
