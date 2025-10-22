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
- [x] Windows, linux, and macOS wheel builds
- [ ] STMatch removed (FMM is primary algorithm)

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
