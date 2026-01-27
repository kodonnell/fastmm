# fastmm

fastmm is a fast (C++) map-matching library for python with no dependencies, and the ability to interpolate time on the match, not just position.

It's based on a desire to map match a lot of vehicle trace data quickly, without the infrastructure to spin up OSRM / Valhalla. (And this is probably faster as there's no IPC ... ?)

It is based on <https://github.com/cyang-kth/fmm> but updated to:

- Remove GDAL/OGR dependencies - networks are created programmatically from Python
- Include Python helper classes for automatic trajectory splitting and time interpolation
- Be buildable on Windows/Linux/Mac with modern tooling
- Focus on Python packaging with distributable wheels
- Remove STMatch - we'll focus on FMM for now

**Status:**

- [ ] Tested ... = )
- [ ] MapMatcher helper class with auto-splitting and time interpolation
- [x] FASTMM algorithm working
- [x] Python API for network creation and matching
- [x] Windows, linux, and macOS wheel builds


## Installation

```bash
pip install fastmm
```

## TODO

- Bring in extra python code.
- Get test working in python.
- If not found in UBODT, instead of bailing, do a normal djikstra lookup.
- Need to check reverse tolerance - on our edges, they're all directed, so we probably shouldn't allow reversing. This causes errors when we're parsing - if you reverse on the same edge, the geometry gets flipped (I think - line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, end_offset); goes backward?), which then messes with our python post-processing of associating time as the segment start/stop are now the edge stop/start, not the other way round. We could add a reversed flag to the edge? That would help. For now, just don't have a reverse tolerance.
- Could move the journey splitting (e.g. when unmatched candidate or points too far apart) into the C++ code here. Would be more optimal as a) C++, and b) don't need to repeat candidate lookup etc.
- Improve serialization of UBODT to be cross-platform.
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
- **Reference speed** represents typical vehicle speed for straight-line travel; used to calculate expected time between GPS points
- Both modes use the same emission probability (based on GPS accuracy) but differ in transition probability calculation
