"""
Simple test to verify fastmm module works correctly.
"""

import sys

sys.path.insert(0, "build/python/pybind11/Release")

import fastmm

print("=" * 60)
print("Testing FASTMM Module")
print("=" * 60)

# Test 1: Create empty network
print("\n1. Creating empty network...")
network = fastmm.Network()
print(f"   ✓ Empty network created")
print(f"   Edges: {network.get_edge_count()}, Nodes: {network.get_node_count()}")

# Test 2: Add edges
print("\n2. Adding edges...")
line = fastmm.LineString()
line.add_point(0.0, 0.0)
line.add_point(100.0, 0.0)
network.add_edge(edge_id=1, source=1, target=2, geom=line)

line2 = fastmm.LineString()
line2.add_point(100.0, 0.0)
line2.add_point(100.0, 100.0)
network.add_edge(edge_id=2, source=2, target=3, geom=line2)

print(f"   ✓ Added 2 edges")
print(f"   Edges: {network.get_edge_count()}, Nodes: {network.get_node_count()}")

# Test 3: Build rtree
print("\n3. Building spatial index...")
network.build_rtree_index()
print(f"   ✓ Spatial index built")

# Test 4: Create graph
print("\n4. Creating network graph...")
graph = fastmm.NetworkGraph(network)
print(f"   ✓ Graph created")

# Test 5: Test LineString from WKT
print("\n5. Testing LineString from WKT...")
wkt_line = fastmm.LineString.from_wkt("LINESTRING (100 100, 0 100)")
print(f"   ✓ LineString created from WKT")
print(f"   Points: {wkt_line.get_num_points()}, Length: {wkt_line.get_length():.2f}")

# Test 6: Test Point creation
print("\n6. Testing Point creation...")
point = fastmm.Point(50.0, 50.0)
print(f"   ✓ Point created: ({point.get_x()}, {point.get_y()})")

# Test 7: Test Trajectory
print("\n7. Testing Trajectory creation...")
trajectory = fastmm.Trajectory.from_xyt_tuples(
    1,
    [
        (10.0, 0.0, 0.0),
        (50.0, 0.0, 5.0),
        (90.0, 0.0, 10.0),
    ],
)
print(f"   ✓ Trajectory created with {len(trajectory)} points")

print("\n" + "=" * 60)
print("All tests passed! ✓")
print("=" * 60)
