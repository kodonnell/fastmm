"""
Complete example showing network creation, UBODT generation, and map matching.
"""
import sys
sys.path.insert(0, "build/python/pybind11/Release")

from pathlib import Path
import fmm

print("=" * 70)
print("FMM Complete Example: Network Creation and Map Matching")
print("=" * 70)

# Step 1: Create a road network
print("\n1. Creating road network...")
network = fmm.Network()

# Create a simple grid: 4 edges forming a square
edges = [
    (1, 1, 2, [(0, 0), (100, 0)]),      # Bottom edge
    (2, 2, 3, [(100, 0), (100, 100)]),  # Right edge  
    (3, 3, 4, [(100, 100), (0, 100)]),  # Top edge
    (4, 4, 1, [(0, 100), (0, 0)]),      # Left edge
]

for edge_id, source, target, coords in edges:
    line = fmm.LineString()
    for x, y in coords:
        line.add_point(float(x), float(y))
    network.add_edge(edge_id=edge_id, source=source, target=target, geom=line)

network.build_rtree_index()
print(f"   Network: {network.get_edge_count()} edges, {network.get_node_count()} nodes")

# Step 2: Create network graph and UBODT
print("\n2. Creating network graph...")
graph = fmm.NetworkGraph(network)

cache_dir = Path("./cache")
cache_dir.mkdir(exist_ok=True)
ubodt_path = cache_dir / "ubodt-example.bin"

if not ubodt_path.exists():
    print(f"   Generating UBODT (this may take a moment)...")
    ubodt_gen = fmm.UBODTGenAlgorithm(network, graph)
    ubodt_gen.generate_ubodt(str(ubodt_path), delta=5000.0)
else:
    print(f"   Using existing UBODT from {ubodt_path}")

ubodt = fmm.UBODT.read_ubodt_binary(str(ubodt_path), multiplier=50000)

# Step 3: Create matcher
print("\n3. Creating map matcher...")
matcher = fmm.FastMapMatch(network, graph, ubodt)

# Step 4: Create and match a trajectory
print("\n4. Matching trajectory...")
# Trajectory along the bottom edge with some GPS noise
trajectory = fmm.Trajectory.from_xyt_tuples(0, [
    (10.0, 1.0, 0.0),    # Near start of edge 1
    (50.0, -2.0, 5.0),   # Middle of edge 1 (with noise)
    (90.0, 0.5, 10.0),   # Near end of edge 1
])

config = fmm.FastMapMatchConfig(
    k=4,
    candidate_search_radius=10.0,
    gps_error=5.0,
    reverse_tolerance=0.0
)

result = matcher.pymatch_trajectory(trajectory, config)

# Step 5: Display results
print("\n5. Match Results:")
print(f"   Error code: {result.error_code}")

if result.error_code == fmm.MatchErrorCode.SUCCESS:
    print(f"   ✓ Match successful!")
    print(f"   Number of segments: {len(result.segments)}")
    
    for i, segment in enumerate(result.segments):
        print(f"\n   Segment {i}:")
        print(f"     From: ({segment.p0.x:.2f}, {segment.p0.y:.2f}) at t={segment.p0.t:.2f}")
        print(f"     To:   ({segment.p1.x:.2f}, {segment.p1.y:.2f}) at t={segment.p1.t:.2f}")
        print(f"     Edges traversed: {len(segment.edges)}")
        
        for edge in segment.edges:
            print(f"       - Edge {edge.edge_id}: {len(edge.points)} matched points")
            # Show first few points
            for j, point in enumerate(edge.points[:3]):
                print(f"         Point {j}: ({point.x:.2f}, {point.y:.2f}), "
                      f"offset={point.edge_offset:.2f}, "
                      f"dist={point.cumulative_distance:.2f}")
else:
    print(f"   ✗ Match failed with error: {result.error_code}")
    if result.unmatched_candidate_indices:
        print(f"   Unmatched points: {result.unmatched_candidate_indices}")

print("\n" + "=" * 70)
print("Example complete!")
print("=" * 70)
