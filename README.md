# fastmm

fastmm is a fast (C++) map-matching library for python with no dependencies, and the ability to interpolate time on the match, not just position.

It's based on a desire to map match a lot of vehicle trace data quickly, without the infrastructure to spin up OSRM / Valhalla. (And this is probably faster as there's no IPC ... ?)

that's designed to be fast, can run on windows, and simpler than spinning up OSRM / Valhalla options.

It is based on <https://github.com/cyang-kth/fmm> but updated to:

- Remove GDAL/OGR dependencies - networks are created programmatically from Python
- Include Python helper classes for automatic trajectory splitting and time interpolation
- Be buildable on Windows with modern tooling
- Focus on Python packaging with distributable wheels

**Status:**
- [x] FASTMM algorithm working
- [x] Python API for network creation and matching
- [x] MapMatcher helper class with auto-splitting and time interpolation
- [x] Windows wheel builds
- [ ] STMatch removed (FMM is primary algorithm)

## Installation

### From Pre-built Wheel

```bash
pip install dist/fastmm-<version>-py3-none-any.whl
```

### Building from Source (Windows)

**Prerequisites:**
- Visual Studio 2022 with C++ tools
- CMake 3.5+
- Python 3.8+
- Conda/Mamba (for Boost and OpenMP)

**Steps:**

1. **Set up environment:**
```powershell
# Create and activate conda environment
conda create -n fastmm python=3.11
conda activate fastmm

# Install dependencies
conda install boost openmp

# Install Python build tools
pip install build wheel setuptools setuptools-scm
```

2. **Build the wheel:**
```powershell
# Simply run the build script
.\build_wheel.ps1
```

The wheel will be created in `dist/` directory.

3. **Optional - Create a versioned release:**

By default, the build creates a development version (e.g., `0.1.dev2`). To create a proper release version:

```powershell
# Tag the release
git tag v0.2.0 -m "Release version 0.2.0"

# Build (will use the tag for version)
.\build_wheel.ps1
```

Version numbering is automatic via `setuptools-scm` based on git tags.

## Usage

```python
import fastmm
from fastmm import Network, MapMatcher, UBODT, FastMapMatchConfig

# Create network programmatically
network = Network()

# Add edges with geometries
for edge_id, edge_data in your_edges.items():
    geom = fastmm.LineString()
    for x, y in edge_data['coordinates']:
        geom.add_point(fastmm.Point(x, y))
    network.add_edge(edge_id, source_node, target_node, geom)

# Build spatial index
network.build_rtree_index()

# Configure and run matching
config = FastMapMatchConfig(gps_error=5.0, radius=50.0)
matcher = MapMatcher(network, config)

# Match trajectory with automatic handling of errors
result = matcher.match_gps(trajectory_df, timestamps_col='timestamp')
```

See `example_matching.py` for complete examples.

## TODO

- Bring in extra python code.
- check tests work.
- Set up builds so boost etc. is bundled into whl, and cibuildwheel for linux/mac/windows.
- If not found in UBODT, instead of bailing, do a normal djikstra lookup.
- Need to check reverse tolerance - on our edges, they're all directed, so we probably shouldn't allow reversing. This causes errors when we're parsing - if you reverse on the same edge, the geometry gets flipped (I think - line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, end_offset); goes backward?), which then messes with our python post-processing of associating time as the segment start/stop are now the edge stop/start, not the other way round. We could add a reversed flag to the edge? That would help. For now, just don't have a reverse tolerance.
- Could move the journey splitting (e.g. when unmatched candidate or points too far apart) into the C++ code here. Would be more optimal as a) C++, and b) don't need to repeat candidate lookup etc.
- Improve serialization of UBODT to be cross-platform.

### Custom costs

To implement, just update the NetworkGraph construction, and set g[e].length = edge.cost (where you read edge in from the read_ogr_file etc.). The change transition probability to

```
// double tp = TransitionGraph::calculate_transition_probability(shortest_path_distance, euclidean_distance);
double tp = exp(-0.01 * shortest_path_distance); //
```

 Seems to work:

- set cost to 1 and it minimizes the number of edges
- set cost to edge length, and gives similar result (as both the methods minimise distance - only differences is the previous method is using euclidean distance between candidates maybe, not matched points? would only matter with dense points).
- can prevent some edges being used by manually bumping up their cost (tested on one road by cost *= 100 for those edges - worked, it avoided those edges).
- Untested:
  - time based: if we have speed on all edges, set cost = distance / speed.
  - use road hierachy or similar - prioritise main highways. More useful for addinsight.
