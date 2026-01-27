# fastmm

fastmm is a fast (C++) map-matching library for python with no dependencies, and the ability to interpolate time on the match (not just position), and also match as much as possible of the GPS trace (not just fail if a single point is wonky).

It's based on a desire to map match a lot of vehicle trace data quickly, without the infrastructure to spin up OSRM / Valhalla. (And this is probably faster as there's no IPC ... ?)

It is based on <https://github.com/cyang-kth/fmm> but updated to:

- Remove GDAL/OGR dependencies - networks are created programmatically from Python
- Include Python helper classes for automatic trajectory splitting and time interpolation
- Be buildable on Windows/Linux/Mac with modern tooling
- Focus on Python packaging with distributable wheels
- Remove STMatch - we'll focus on FMM for now

**Status:**

- [x] Tested ... = )
- [x] MapMatcher helper class with auto-splitting and time interpolation
- [x] FASTMM algorithm working
- [x] Python API for network creation and matching
- [x] Windows, linux, and macOS wheel builds


## Installation

```bash
pip install fastmm
```

## Usage

### Basic Map Matching

```python
import fastmm

# Create network
network = fastmm.Network()
network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
network.build_rtree_index()

# Build routing graph and UBODT
graph = fastmm.NetworkGraph(network)
ubodt_gen = fastmm.UBODTGenAlgorithm(network, graph)
ubodt_gen.generate_ubodt("ubodt.bin", delta=1000)
ubodt = fastmm.UBODT.read_ubodt_binary("ubodt.bin", multiplier=50000)

# Create matcher and config
matcher = fastmm.FastMapMatch(network, graph, ubodt)
config = fastmm.FastMapMatchConfig(k=8, candidate_search_radius=50, gps_error=50)

# Match a trajectory
trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (50, 0), (150, 0)])
result = matcher.pymatch_trajectory(trajectory, config)

if result.error_code == fastmm.MatchErrorCode.SUCCESS:
    for segment in result.segments:
        print(f"Segment from {segment.p0} to {segment.p1}")
        for edge in segment.edges:
            print(f"  Edge {edge.edge_id} with {len(edge.points)} points")
```

### Automatic Trajectory Splitting

For trajectories that might have gaps or failures, use `pymatch_trajectory_split()` which automatically handles:
- Points with no nearby road candidates (e.g., in tunnels, off-network)
- Disconnected route segments (e.g., GPS jumps, teleportation)
- Returns only the successfully matched portions

```python
# Same network/matcher setup as above...

# Match with automatic splitting
trajectory = fastmm.Trajectory.from_xy_tuples(1, [
    (10, 0),    # Point 0
    (50, 0),    # Point 1
    (100, 500), # Point 2 - far from network, will be skipped
    (150, 0),   # Point 3
    (180, 0),   # Point 4
])
result = matcher.pymatch_trajectory_split(trajectory, config)

# Process successful sub-trajectories
for sub in result.subtrajectories:
    print(f"Matched points {sub.start_index} to {sub.end_index}")
    for segment in sub.segments:
        print(f"  Segment with {len(segment.edges)} edges")
```

**Benefits of split matching:**
- **Performance**: Candidate search done once for all points, then reused across sub-trajectories
- **Automatic recovery**: Continues matching after failures instead of giving up
- **Simplicity**: No manual trajectory splitting or error handling needed

## TODO

- what is delta for ubodt FASTEST?
- Can we auto-compute multiplier? Can we hide UBODT completely from user?
- Ensure UBODT mode matches network when loaded. Improve serialization of UBODT to be cross-platform.
- Bring in extra python code.
- Get test working in python.
- If not found in UBODT, instead of bailing, do a normal djikstra lookup.
- Need to check reverse tolerance - on our edges, they're all directed, so we probably shouldn't allow reversing. This causes errors when we're parsing - if you reverse on the same edge, the geometry gets flipped (I think - line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, end_offset); goes backward?), which then messes with our python post-processing of associating time as the segment start/stop are now the edge stop/start, not the other way round. We could add a reversed flag to the edge? That would help. For now, just don't have a reverse tolerance.
- Specify versions for build libs (e.g. cibuildwheel).

## Routing Modes: SHORTEST vs FASTEST

FastMM supports two routing modes that affect how map matching selects the most likely path:

### SHORTEST Mode (Distance-based)

Uses distance as the routing metric. This is the default mode and matches trajectories based on spatial proximity.

```python
import fastmm

# Create network with SHORTEST mode (default)
network = fastmm.Network()
network.add_edge(edge_id=1, source=1, target=2, geom=linestring)
network.build_rtree_index()

# Create graph for distance-based routing
graph = fastmm.NetworkGraph(network, mode=fastmm.TransitionMode.SHORTEST)

# Configure matcher (SHORTEST is default)
config = fastmm.FastMapMatchConfig(
    k=8,
    candidate_search_radius=50,
    gps_error=50,
    transition_mode=fastmm.TransitionMode.SHORTEST
)
```

**Transition Probability Calculation (SHORTEST):**
```
tp = min(euclidean_dist, path_dist) / max(euclidean_dist, path_dist)
```

This compares the straight-line distance between GPS points to the network path distance. Higher probability when the path closely follows the GPS trajectory.

**Practical Effect:** Prefers routes that minimize total distance, even if they involve slower roads. Best for pedestrian tracking, cycling, or when speed data is unavailable.

### FASTEST Mode (Time-based)

Uses travel time as the routing metric. Requires speed values on all edges.

```python
import fastmm

# Create network with speed on edges
network = fastmm.Network()
network.add_edge(edge_id=1, source=1, target=2, geom=linestring, speed=50.0)  # speed in units/time
network.add_edge(edge_id=2, source=2, target=3, geom=linestring2, speed=30.0)
network.build_rtree_index()

# Create graph for time-based routing
graph = fastmm.NetworkGraph(network, mode=fastmm.TransitionMode.FASTEST)

# Configure matcher with reference speed
config = fastmm.FastMapMatchConfig(
    k=8,
    candidate_search_radius=50,
    gps_error=50,
    transition_mode=fastmm.TransitionMode.FASTEST,
    reference_speed=40.0  # Expected travel speed (required for FASTEST)
)
```

**Transition Probability Calculation (FASTEST):**
```
expected_time = euclidean_dist / reference_speed
actual_time = path_time (sum of segment_length / segment_speed)
tp = min(expected_time, actual_time) / max(expected_time, actual_time)
```

This compares the expected time (if traveling straight at reference speed) to the actual time needed to traverse the network path.

**Practical Effect:** Prefers faster routes (highways, arterials) over slower alternatives, even if slightly longer. Best for vehicle tracking where drivers optimize for time, not distance.

### Important Notes

- **FASTEST mode requires speed on ALL edges** - the system will throw an error at graph construction if any edge lacks a speed value
- **Separate UBODTs needed** - Generate separate UBODT files for each mode, as they store different costs (distance vs time)
- Both modes use the same emission probability (based on GPS accuracy) but differ in transition probability calculation

### Understanding Reference Speed

**What is reference_speed?**

The `reference_speed` parameter represents the expected average speed at which a vehicle would travel in a straight line between GPS observation points, **if it could ignore the road network**. It's used exclusively in FASTEST mode to calculate expected travel times.

**How it affects matching:**

In FASTEST mode, the transition probability between two GPS points compares:
- **Expected time**: How long would it take to travel the straight-line distance at `reference_speed`
- **Actual time**: How long does the matched network path take (sum of edge_length/edge_speed for each edge)

The probability is higher when these times are similar: `tp = min(expected, actual) / max(expected, actual)`

**Choosing the right value:**

- **Lower values (e.g., 20-30)**:
  - Makes the algorithm expect slower straight-line travel
  - Increases expected_time, making actual network paths look relatively faster
  - **Effect**: Encourages following the road network more closely, even if it means taking detours
  - **Good for**: Dense urban areas, congested traffic, routes with many turns

- **Higher values (e.g., 60-80)**:
  - Makes the algorithm expect faster straight-line travel
  - Decreases expected_time, making actual network paths look relatively slower
  - **Effect**: More tolerant of GPS points that "cut corners" or skip parts of the route
  - **Good for**: Highway driving, rural areas, trajectories with sparse GPS sampling

- **Typical values (e.g., 40-50)**:
  - Match the average actual vehicle speed in your dataset
  - A good starting point for mixed urban/suburban driving
  - Should be close to the typical speeds on your network edges

**Practical Example:**

Imagine two GPS points 100 meters apart (straight-line):
- Network path: follows roads for 120 meters through edges with average speed 50
- Actual time on network: 120m / 50 = 2.4 time units

With `reference_speed=40`:
- Expected time: 100m / 40 = 2.5 time units
- Transition probability: min(2.5, 2.4) / max(2.5, 2.4) = 2.4/2.5 = **0.96** (high - good match!)

With `reference_speed=60`:
- Expected time: 100m / 60 = 1.67 time units
- Transition probability: min(1.67, 2.4) / max(1.67, 2.4) = 1.67/2.4 = **0.70** (lower - penalizes this path)

**Rule of thumb**: Set `reference_speed` close to the average speed in your network. If matching is too "sticky" to routes (not handling shortcuts well), increase it. If matching is too loose (taking implausible shortcuts), decrease it.
