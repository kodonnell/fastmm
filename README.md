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

## Quick Start (Recommended)

The simplest way to use fastmm is with the high-level `MapMatcher` class:

```python
import fastmm

# Create and populate network
network = fastmm.Network()
network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)])
network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)])
network.finalize()

# Create matcher with automatic UBODT caching (SHORTEST mode - distance-based)
matcher = fastmm.MapMatcher(
    network=network,
    mode=fastmm.TransitionMode.SHORTEST,
    max_distance_between_candidates=300.0,  # meters (2-3x expected GPS spacing)
    cache_dir="./cache"  # UBODT cached here, auto-regenerated if network changes
)

# Match a trajectory
config = fastmm.FastMapMatchConfig(k=8, candidate_search_radius=50, gps_error=50)
trajectory = fastmm.Trajectory.from_xy_tuples(1, [(10, 0), (50, 0), (150, 0)])
result = matcher.model.pymatch_trajectory(trajectory, config)

if result.error_code == fastmm.MatchErrorCode.SUCCESS:
    for segment in result.segments:
        print(f"Segment from {segment.p0} to {segment.p1}")
        for edge in segment.edges:
            print(f"  Edge {edge.edge_id} with {len(edge.points)} points")
```

### FASTEST Mode (Time-Based Routing)

For time-based routing (requires speed on all edges):

```python
# Add edges with speed information
network = fastmm.Network()
network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=13.9)  # 50 km/h
network.add_edge(2, source=2, target=3, geom=[(100, 0), (200, 0)], speed=13.9)
network.finalize()

# Create matcher with FASTEST mode
matcher = fastmm.MapMatcher(
    network=network,
    mode=fastmm.TransitionMode.FASTEST,
    max_time_between_candidates=20.0,  # seconds (2-3x expected travel time between GPS points)
    cache_dir="./cache"
)

# The rest is the same...
```

## TODO

- tidy readme
- xyt if no t provided has t = 0, when probably should be null.
- test the time apportioning.
- unused
- If not found in UBODT, instead of bailing, do a normal djikstra lookup.
- max_distance_between_candidates is not a hard limit in UBODT ... I think. Test this, and if needed, add an extra check.
- Specify versions for build libs (e.g. cibuildwheel).

### Automatic Trajectory Splitting

For trajectories that might have gaps or failures, use `match()` which automatically handles:
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

## Understanding Delta Parameters

The `delta` parameter (called `max_distance_between_candidates` or `max_time_between_candidates` in `MapMatcher`) controls the maximum routing cost for precomputed paths in the UBODT table:

### SHORTEST Mode (Distance-Based)
- **Units**: Same as your network (typically meters)
- **Meaning**: Maximum road network distance between GPS points
- **Recommendation**: 2-3x your expected maximum distance between consecutive GPS points
- **Example**: If GPS points are ~100m apart, use delta=300m

### FASTEST Mode (Time-Based)
- **Units**: Seconds
- **Meaning**: Maximum travel time between GPS points
- **Recommendation**: 2-3x your expected maximum travel time between GPS points
- **Example**: For 200m spacing at 50km/h expected speed: 200m ÷ (50,000m/3600s) ≈ 14.4s → use delta=40s

**Trade-offs**:
- **Larger delta**: Better matching quality (more routing options), but larger file size and slower generation
- **Smaller delta**: Faster generation and smaller files, but may fail to find paths between distant GPS points

## Understanding Reverse Tolerance

The `reverse_tolerance` parameter handles GPS measurement noise that causes slight backward movement on the **same edge**. This is different from edge direction:

### How It Works

**Edge Traversal:** The graph uses **directed edges**. Dijkstra routing always respects edge direction (source → target). For OSM data with bidirectional roads, you should have two edges (one per direction).

**Same-Edge Positioning:** When two consecutive GPS points match to the **same edge** with the second point having a lower offset than the first (backward movement), `reverse_tolerance` controls whether this is allowed:

```python
# Example: GPS noise causes apparent backward movement
GPS Point 1 → Edge 1 at offset=80m (80% along A→B)
GPS Point 2 → Edge 1 at offset=50m (50% along A→B)

# Without reverse_tolerance (0.0):
# - Transition has infinite cost → rejected
# - Algorithm may match Point 2 to opposite-direction edge (creating fake U-turn)
# - Or Point 2 gets skipped in split mode

# With reverse_tolerance=40 (40m in these units):
# - Backward movement = 80m - 50m = 30m
# - 30m < 40m ✅ Allowed with cost=0
```

### The Reversed Flag

When backward movement is allowed (within tolerance), the `reversed` flag indicates this occurred. **The geometry is automatically corrected** to always go forward (from lower to higher offset), so you don't need to handle backward linestrings:

```python
for segment in result.segments:
    for edge in segment.edges:
        if edge.reversed:
            # Geometry has been auto-corrected to go forward
            # But you know GPS moved backward on this edge
            # May want to flag this for quality control
            print(f"Edge {edge.edge_id} had backward GPS movement (now corrected)")

        # All edges have forward geometry regardless of reversed flag
        # edge.points always go from lower to higher offset
        for point in edge.points:
            print(f"  Offset: {point.edge_offset}, Position: ({point.x}, {point.y})")
```

**What the `reversed` flag means:**
- `reversed=True`: GPS moved backward (offset1 > offset2), geometry was auto-corrected to go forward
- `reversed=False`: GPS moved forward normally (offset1 <= offset2)

**No special handling needed** - the geometry is always correct. Use the flag for:
- Quality control (detecting erratic GPS behavior)
- Statistics (counting backward movements)
- Debugging (understanding matching behavior)
```

### Recommendations

**For OSM Networks (bidirectional edges):**
- Use `reverse_tolerance=0.0` (default) to avoid fake U-turns
- GPS noise is better handled by `gps_error` and trajectory splitting
- Real backward movement should match to the opposite-direction edge

**For Stationary/Slow Vehicles:**
- Consider `reverse_tolerance=20` (20m assuming in Euclidean system)  to handle GPS jitter
- Check `edge.reversed` flag in Python to handle geometry correctly
- May need post-processing to detect oscillating matches

## Routing Modes: SHORTEST vs FASTEST

FastMM supports two routing modes that affect how map matching selects the most likely path. The mode must be specified when creating the graph and UBODT, and is automatically validated when using `MapMatcher`.

### SHORTEST Mode (Distance-based)

Uses distance as the routing metric. This is the default mode and matches trajectories based on spatial proximity.

```python
import fastmm

# Create network with SHORTEST mode (default)
network = fastmm.Network()
network.add_edge(edge_id=1, source=1, target=2, geom=linestring)
network.finalize()

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
network.finalize()

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
  - `MapMatcher` handles this automatically through cache file naming
  - Manual API users must generate separate UBODT files with different `delta` values and filenames
- **Mode validation** - New UBODT files (v1+) include metadata that validates mode compatibility
  - `MapMatcher` automatically validates mode when loading cached UBODT
  - Manual API: use validation via `UBODT.get_mode()` after `read_ubodt()`
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

