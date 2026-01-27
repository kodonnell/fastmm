"""
Combined example demonstrating shortest vs fastest path matching.

Network Diagram:
================================================================================

              (100,10) ------------- (200,10)
                  |      100 km/h       |
                  |      Edge 4         |
                  |     (100 units)     |
              100 km/h              100 km/h
               Edge 3                Edge 5
            (10 units)             (10 units)
                  |                     |
    A ----------- B ------------------- C ----------- D
  (0,0)        (100,0)               (200,0)      (300,0)
         50 km/h         50 km/h           50 km/h
         Edge 1           Edge 2            Edge 6
        (100 units)     (100 units)       (100 units)

Route Options from B to C:
---------------------------
1. DIRECT route (Edge 2):
   - Distance: 100 units
   - Speed: 50 km/h
   - Travel time: 2 time units
   - SHORTER but SLOWER

2. DETOUR route (Edges 3→4→5):
   - Distance: 10 + 100 + 10 = 120 units
   - Speed: 100 km/h
   - Travel time: 1.2 time units
   - LONGER but FASTER

Expected behavior:
- SHORTEST routing → chooses Edge 2 (direct)
- FASTEST routing → chooses Edges 3,4,5 (detour)

================================================================================
"""

import sys

sys.path.insert(0, "build/python/pybind11/Release")

from pathlib import Path

from fastmm import (
    UBODT,
    FastMapMatch,
    FastMapMatchConfig,
    LineString,
    MatchErrorCode,
    Network,
    NetworkGraph,
    Trajectory,
    TransitionMode,
    UBODTGenAlgorithm,
)

print("=" * 80)
print("FASTMM Combined Example: Shortest vs Fastest Path")
print("=" * 80)

# Step 1: Create the road network
print("\n1. Creating road network...")
network = Network()

# Edge 1: A->B along Y=0 (node 1 to node 2)
print("   Adding Edge 1: A->B (Y=0, 50 km/h)...")
line_ab = LineString()
line_ab.add_point(0, 0)
line_ab.add_point(100, 0)
network.add_edge(edge_id=1, source=1, target=2, geom=line_ab, speed=50)

# Edge 2: B->C direct along Y=0 (node 2 to node 3)
print("   Adding Edge 2: B->C direct (Y=0, 50 km/h)...")
line_bc_direct = LineString()
line_bc_direct.add_point(100, 0)
line_bc_direct.add_point(200, 0)
network.add_edge(edge_id=2, source=2, target=3, geom=line_bc_direct, speed=50)

# Edge 3: B->up (100,0) to (100,10) (node 2 to node 4)
print("   Adding Edge 3: B->up (100 km/h)...")
line_b_up = LineString()
line_b_up.add_point(100, 0)
line_b_up.add_point(100, 10)
network.add_edge(edge_id=3, source=2, target=4, geom=line_b_up, speed=100)

# Edge 4: across top (100,10) to (200,10) (node 4 to node 5)
print("   Adding Edge 4: across top (100 km/h)...")
line_detour = LineString()
line_detour.add_point(100, 10)
line_detour.add_point(200, 10)
network.add_edge(edge_id=4, source=4, target=5, geom=line_detour, speed=100)

# Edge 5: down to C (200,10) to (200,0) (node 5 to node 3)
print("   Adding Edge 5: down->C (100 km/h)...")
line_c_down = LineString()
line_c_down.add_point(200, 10)
line_c_down.add_point(200, 0)
network.add_edge(edge_id=5, source=5, target=3, geom=line_c_down, speed=100)

# Edge 6: C->D along Y=0 (node 3 to node 6)
print("   Adding Edge 6: C->D (Y=0, 50 km/h)...")
line_cd = LineString()
line_cd.add_point(200, 0)
line_cd.add_point(300, 0)
network.add_edge(edge_id=6, source=3, target=6, geom=line_cd, speed=50)

network.build_rtree_index()
print(f"   ✓ Network created: {network.get_edge_count()} edges, {network.get_node_count()} nodes")

# Step 2: Create network graphs for SHORTEST and FASTEST routing
print("\n2. Creating network graphs for SHORTEST and FASTEST routing...")
graph_shortest = NetworkGraph(network, TransitionMode.SHORTEST)
graph_fastest = NetworkGraph(network, TransitionMode.FASTEST)
print("   ✓ Graphs created")

# Step 3: Generate UBODTs
cache_dir = Path("./.cache")
cache_dir.mkdir(exist_ok=True)

print("\n3. Generating UBODTs...")
ubodt_shortest_path = cache_dir / "ubodt-combined-shortest.bin"
ubodt_fastest_path = cache_dir / "ubodt-combined-fastest.bin"

print("   Generating UBODT for SHORTEST path...")
ubodt_gen_shortest = UBODTGenAlgorithm(network, graph_shortest)
ubodt_gen_shortest.generate_ubodt(str(ubodt_shortest_path), delta=5000)

print("   Generating UBODT for FASTEST path...")
ubodt_gen_fastest = UBODTGenAlgorithm(network, graph_fastest)
ubodt_gen_fastest.generate_ubodt(str(ubodt_fastest_path), delta=5000)

ubodt_shortest = UBODT.read_ubodt_binary(str(ubodt_shortest_path), multiplier=50000)
ubodt_fastest = UBODT.read_ubodt_binary(str(ubodt_fastest_path), multiplier=50000)
print("   ✓ UBODTs loaded")

# Step 4: Create matchers
print("\n4. Creating map matchers...")
matcher_shortest = FastMapMatch(network, graph_shortest, ubodt_shortest)
matcher_fastest = FastMapMatch(network, graph_fastest, ubodt_fastest)
print("   ✓ Matchers created")

# Step 5: Create GPS trajectory along A->B->?->C->D
print("\n5. Creating GPS trajectory...")
# GPS points along A->B and C->D, with ambiguous middle section
trajectory = Trajectory.from_xyt_tuples(
    1,
    [
        # Along A->B (Edge 1)
        (25, 0, 0),
        (75, 0, 5),
        # Middle section - ambiguous (could match to either route)
        (150, 5, 10),  # Between direct and detour routes
        # Along C->D (Edge 6)
        (225, 0, 15),
        (275, 0, 20),
    ],
)
print(f"   ✓ Trajectory with {len(trajectory)} GPS points")

# Step 6: Configure matching
config_shortest = FastMapMatchConfig(
    k=8, candidate_search_radius=15, gps_error=5, reverse_tolerance=0, transition_mode=TransitionMode.SHORTEST
)

config_fastest = FastMapMatchConfig(
    k=8,
    candidate_search_radius=15,
    gps_error=5,
    reverse_tolerance=0,
    transition_mode=TransitionMode.FASTEST,
    reference_speed=50,
)

# Step 7: Match with SHORTEST routing
print("\n" + "=" * 80)
print("MATCHING WITH SHORTEST ROUTING (distance-based)")
print("=" * 80)

result_shortest = matcher_shortest.pymatch_trajectory(trajectory, config_shortest)

if result_shortest.error_code == MatchErrorCode.SUCCESS:
    print("✓ Match successful!")
    all_edges = []
    for segment in result_shortest.segments:
        for edge in segment.edges:
            all_edges.append(edge.edge_id)

    # Remove consecutive duplicates
    route = []
    for edge_id in all_edges:
        if not route or route[-1] != edge_id:
            route.append(edge_id)

    print(f"\nMatched route: {' → '.join(map(str, route))}")

    if route == [1, 2, 6]:
        print("✓ CORRECT: Used DIRECT route (Edge 2) as expected for SHORTEST!")
    elif 3 in route and 4 in route and 5 in route:
        print("✗ UNEXPECTED: Used DETOUR route instead of direct")
    else:
        print(f"? Route doesn't match expected patterns")
else:
    print(f"✗ Match failed: {result_shortest.error_code}")

# Step 8: Match with FASTEST routing
print("\n" + "=" * 80)
print("MATCHING WITH FASTEST ROUTING (time-based)")
print("=" * 80)

result_fastest = matcher_fastest.pymatch_trajectory(trajectory, config_fastest)

if result_fastest.error_code == MatchErrorCode.SUCCESS:
    print("✓ Match successful!")
    all_edges = []
    for segment in result_fastest.segments:
        for edge in segment.edges:
            all_edges.append(edge.edge_id)

    # Remove consecutive duplicates
    route = []
    for edge_id in all_edges:
        if not route or route[-1] != edge_id:
            route.append(edge_id)

    print(f"\nMatched route: {' → '.join(map(str, route))}")

    if route == [1, 3, 4, 5, 6]:
        print("✓ CORRECT: Used DETOUR route (Edges 3→4→5) as expected for FASTEST!")
    elif 2 in route:
        print("✗ UNEXPECTED: Used DIRECT route instead of detour")
    else:
        print(f"? Route doesn't match expected patterns")
else:
    print(f"✗ Match failed: {result_fastest.error_code}")

# Summary
print("\n" + "=" * 80)
print("SUMMARY")
print("=" * 80)
print("\nExpected behavior:")
print("  SHORTEST routing → Edge 2 (direct, 100 units, slower)")
print("  FASTEST routing  → Edges 3→4→5 (detour, 120 units, faster)")
print("\nThis demonstrates how routing mode affects path selection when")
print("edge speeds create a tradeoff between distance and travel time.")
print("=" * 80)
