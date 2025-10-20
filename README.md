# pyfmm

pyfmm is a map-matching library for python, that's designed to be fast, can run on windows, and simpler than spinning up OSRM / Valhalla options.

It is based on <https://github.com/cyang-kth/fmm> but updated to:

- Have some extra features (specifically making it easier to interpolate time along matched routes)
- Be updated
- Buildable on windows
- Focus on python, and remove a lot of the other GUI/examples/etc. stuff. At this stage, see the original repo for demonstrations of how it performs etc. - we just want to wrap it nicely in python.

Status:

- Working for FMM
- STMatch might be migrated, but haven't checked.

## Building/installing

My rough note on Windows using conda.

install cmake

activate repo to install into

mamba install boost
$env:BOOST_ROOT = "$env:CONDA_PREFIX\Library"
$env:BOOST_INCLUDEDIR = "$env:CONDA_PREFIX\Library\include"
$env:BOOST_LIBRARYDIR = "$env:CONDA_PREFIX\Library\lib"

mkdir build
cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release --parallel 8

Go to build/python
copy Release/_fmm.pyc to .
Copy ../Release/FMMLIB.dll to .
add this location to sys.path and you should be able to `import fmm`

## TODO

- Bring in extra python code.
- check tests work.
- Set up builds so boost etc. is bundled into whl, and cibuildwheel for linux/mac/windows.
- If not found in UBODT, instead of bailing, do a normal djikstra lookup.
- Need to check reverse tolerance - on our edges, they're all directed, so we probably shouldn't allow reversing. This causes errors when we're parsing - if you reverse on the same edge, the geometry gets flipped (I think - line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, end_offset); goes backward?), which then messes with our python post-processing of associating time as the segment start/stop are now the edge stop/start, not the other way round. We could add a reversed flag to the edge? That would help. For now, just don't have a reverse tolerance.
- Could move the journey splitting (e.g. when unmatched candidate or points too far apart) into the C++ code here. Would be more optimal as a) C++, and b) don't need to repeat candidate lookup etc.

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
