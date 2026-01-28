// fastmm_bindings.cpp: pybind11 bindings for FASTMM
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "core/geometry.hpp"
#include "core/gps.hpp"
#include "network/type.hpp"
#include "network/network.hpp"
#include "network/network_graph.hpp"
#include "mm/mm_type.hpp"
#include "mm/fmm/fmm_algorithm.hpp"
#include "mm/fmm/ubodt_gen_algorithm.hpp"
#include "mm/fmm/ubodt.hpp"

namespace py = pybind11;
using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::NETWORK;
using namespace FASTMM::MM;

PYBIND11_MODULE(fastmm, m)
{
    m.doc() = "Fast Map Matching (FASTMM) Python bindings via pybind11";

    // MatchErrorCode enum
    py::enum_<MatchErrorCode>(m, "MatchErrorCode")
        .value("SUCCESS", MatchErrorCode::SUCCESS, "Matching succeeded")
        .value("CANDIDATES_NOT_FOUND", MatchErrorCode::CANDIDATES_NOT_FOUND, "No candidate edges found for trajectory")
        .value("DISCONNECTED_LAYERS", MatchErrorCode::DISCONNECTED_LAYERS, "Trajectory has disconnected layers")
        .value("INDEX_OUT_OF_BOUNDS", MatchErrorCode::INDEX_OUT_OF_BOUNDS, "Start edge index out of bounds")
        .value("INDEX_OUT_OF_BOUNDS_END", MatchErrorCode::INDEX_OUT_OF_BOUNDS_END, "End edge index out of bounds")
        .value("UNKNOWN_ERROR", MatchErrorCode::UNKNOWN_ERROR, "Unknown error occurred")
        .export_values();

    // TransitionMode enum
    py::enum_<TransitionMode>(m, "TransitionMode")
        .value("SHORTEST", TransitionMode::SHORTEST, "Distance-based routing")
        .value("FASTEST", TransitionMode::FASTEST, "Time-based routing")
        .export_values();

    // Network class
    py::class_<Network>(m, "Network", R"pbdoc(
        A road network consisting of nodes (junctions) and directed edges (road segments).

        The network must be fully constructed (all edges added) before building the spatial
        index. Once the index is built, the network is ready for map matching operations.

        Example:
            >>> network = fastmm.Network()
            >>> network.add_edge(1, source=10, target=20, geom=[(0, 0), (100, 0)], speed=50.0)
            >>> network.finalize()
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Create an empty network.

            Use add_edge() to populate the network with road segments, then call
            finalize() to prepare it for map matching.
        )pbdoc")
        .def("add_edge", [](Network &self, int edge_id, int source, int target, py::list coords, std::optional<double> speed)
             {
            LineString geom;
            for (auto item : coords) {
                if (py::isinstance<py::tuple>(item) && py::len(item) == 2) {
                    py::tuple tup = item.cast<py::tuple>();
                    double x = py::float_(tup[0]);
                    double y = py::float_(tup[1]);
                    geom.add_point(x, y);
                } else {
                    throw std::runtime_error("Each coordinate must be a tuple (x, y)");
                }
            }
            if (geom.get_num_points() < 2) {
                throw std::runtime_error("Edge geometry must have at least 2 points");
            }
            self.add_edge(edge_id, source, target, geom, speed); }, py::arg("edge_id"), py::arg("source"), py::arg("target"), py::arg("geom"), py::arg("speed") = std::nullopt,
             R"pbdoc(
            Add a directed edge (road segment) to the network.

            Each edge must have a unique ID and connects two nodes. The geometry defines
            the spatial path of the edge. Speed is required for FASTEST routing mode.

            Args:
                edge_id: Unique integer identifier for this edge
                source: Node ID where the edge starts
                target: Node ID where the edge ends
                geom: List of (x, y) tuples defining the edge geometry (minimum 2 points)
                speed: Optional speed value (distance units per time unit).
                       Required if using TransitionMode.FASTEST routing.

            Note:
                Call finalize() after adding all edges.

            Example:
                >>> network.add_edge(1, source=1, target=2, geom=[(0, 0), (100, 0)], speed=50.0)
        )pbdoc")
        .def("finalize", &Network::finalize,
             R"pbdoc(
            Build the spatial R-tree index for efficient candidate edge lookup.

            This MUST be called after adding all edges and before creating a NetworkGraph
            or performing any map matching operations. The index enables fast spatial
            queries to find nearby road segments for GPS points.

            Raises:
                RuntimeError: If called on an empty network
        )pbdoc")
        .def("compute_hash", &Network::compute_hash,
             R"pbdoc(
            Compute a hash of the network structure for cache validation.

            The hash is computed from edge count, sampled edge IDs, sources,
            targets, and speeds. It's used to detect network changes and
            invalidate cached UBODT files.

            Returns:
                str: 8-character hexadecimal hash string
        )pbdoc")
        .def("get_edge_count", &Network::get_edge_count, R"pbdoc(
            Get the total number of edges in the network.

            Returns:
                int: Number of edges
        )pbdoc")
        .def("get_node_count", &Network::get_node_count, R"pbdoc(
            Get the total number of nodes in the network.

            Returns:
                int: Number of unique nodes
        )pbdoc");

    // NetworkGraph class
    py::class_<NetworkGraph>(m, "NetworkGraph", R"pbdoc(
        A routing graph built from a Network for path finding.

        The graph representation depends on the chosen routing mode:
        - SHORTEST: Uses edge distances for path costs
        - FASTEST: Uses edge travel times (distance/speed) for path costs

        A separate graph must be created for each routing mode, and FASTEST mode
        requires that all edges have speed values defined.
    )pbdoc")
        .def(py::init<const Network &, TransitionMode>(),
             py::arg("network"),
             py::arg("mode") = TransitionMode::SHORTEST,
             R"pbdoc(
            Create a NetworkGraph from a Network with specified routing mode.

            Args:
                network: Network with edges (must have finalize() called)
                mode: Routing mode - SHORTEST (distance-based) or FASTEST (time-based).
                      Default is SHORTEST.

            Raises:
                RuntimeError: If mode is FASTEST and any edge lacks a speed value

            Note:
                Generate separate UBODT files for each routing mode, as the shortest
                paths differ between distance-based and time-based routing.
        )pbdoc");

    // FastMapMatchConfig class
    py::class_<FastMapMatchConfig>(m, "FastMapMatchConfig", R"pbdoc(
        Configuration parameters for the Fast Map Matching algorithm.

        Controls candidate generation, emission/transition probability calculations,
        and routing behavior. Proper tuning of these parameters significantly affects
        matching quality and performance.
    )pbdoc")
        .def(py::init<int, double, double, double, TransitionMode, std::optional<double>>(),
             py::arg("k") = 8,
             py::arg("candidate_search_radius") = 50,
             py::arg("gps_error") = 50,
             py::arg("reverse_tolerance") = 0.0,
             py::arg("transition_mode") = TransitionMode::SHORTEST,
             py::arg("reference_speed") = std::nullopt,
             R"pbdoc(
            Create map matching configuration.

            Args:
                k: Maximum number of candidate edges to consider per GPS point.
                   Higher values improve matching quality in dense networks but slow
                   performance. Typical: 4-16. Default: 8.

                candidate_search_radius: Maximum distance to search for candidate edges
                   around each GPS point (in coordinate units). Should exceed typical GPS
                   errors. Typical: 30-100 meters. Default: 50.

                gps_error: Expected GPS accuracy (standard deviation in coordinate units).
                   Used in emission probability: P(obs|candidate) ~ exp(-dist²/(2*gps_error²)).
                   Higher values are more tolerant of GPS noise. Typical: 10-100 meters.
                   Default: 50.

                reverse_tolerance: Maximum distance allowed when routing backward along
                   an edge (in coordinate units). Set to 0 to forbid reversing, which is
                   recommended for directed road networks. Default: 0.0.

                transition_mode: Routing cost metric - SHORTEST (distance) or FASTEST (time).
                   Must match the mode used to create NetworkGraph and UBODT. Default: SHORTEST.

                reference_speed: Expected travel speed for straight-line movement between GPS
                   points (distance units per time unit). REQUIRED for FASTEST mode, unused
                   for SHORTEST mode. This represents the typical speed at which vehicles
                   travel directly between points. Lower values encourage sticking to routes,
                   higher values allow more detours. Typical: average vehicle speed like 40-60
                   in urban areas. Default: None.

            Raises:
                ValueError: If transition_mode is FASTEST but reference_speed is None
        )pbdoc")
        .def_readwrite("k", &FastMapMatchConfig::k, "Maximum number of candidate edges per GPS point")
        .def_readwrite("candidate_search_radius", &FastMapMatchConfig::candidate_search_radius,
                       "Search radius for finding candidate edges (coordinate units)")
        .def_readwrite("gps_error", &FastMapMatchConfig::gps_error,
                       "Expected GPS accuracy for emission probability (coordinate units)")
        .def_readwrite("reverse_tolerance", &FastMapMatchConfig::reverse_tolerance,
                       "Maximum distance allowed for reverse routing on edges (coordinate units)")
        .def_readwrite("transition_mode", &FastMapMatchConfig::transition_mode,
                       "Routing mode: SHORTEST (distance-based) or FASTEST (time-based)")
        .def_readwrite("reference_speed", &FastMapMatchConfig::reference_speed,
                       "Expected straight-line travel speed (required for FASTEST mode)");

    // PyMatchPoint struct
    py::class_<PyMatchPoint>(m, "PyMatchPoint", R"pbdoc(
        A matched point along a road edge in the map matching result.

        Represents a specific location on a matched edge, including its position
        relative to the edge start and the cumulative distance from the trajectory start.
    )pbdoc")
        .def_readonly("x", &PyMatchPoint::x, "X coordinate of the matched point")
        .def_readonly("y", &PyMatchPoint::y, "Y coordinate of the matched point")
        .def_readonly("edge_offset", &PyMatchPoint::edge_offset,
                      "Distance from the start of the edge to this point (coordinate units)")
        .def_readonly("cumulative_distance", &PyMatchPoint::cumulative_distance,
                      "Total distance from the trajectory start to this point (coordinate units)")
        .def("__repr__", [](const PyMatchPoint &p)
             { return "<P x=" + fmt::format("{:.1f}", p.x) +
                      " y=" + fmt::format("{:.1f}", p.y) +
                      " edge_offset=" + fmt::format("{:.1f}", p.edge_offset) +
                      " cumulative_distance=" + fmt::format("{:.1f}", p.cumulative_distance) + ">"; });

    // PyMatchSegmentEdge struct
    py::class_<PyMatchSegmentEdge>(m, "PyMatchSegmentEdge", R"pbdoc(
        A matched road edge with interpolated points along the matched path.

        Contains the edge ID and a sequence of matched points representing where
        the trajectory intersects or follows this edge.
    )pbdoc")
        .def_readonly("edge_id", &PyMatchSegmentEdge::edge_id,
                      "ID of the matched road edge from the network")
        .def_readonly("points", &PyMatchSegmentEdge::points,
                      "List of PyMatchPoint objects along this edge, ordered by position")
        .def_readonly("reversed", &PyMatchSegmentEdge::reversed,
                      "True if geometry is reversed (GPS moved backward on same edge due to reverse_tolerance)")
        .def("__repr__", [](const PyMatchSegmentEdge &e)
             { return "<Edge edge_id=" + std::to_string(e.edge_id) + " with " + std::to_string(e.points.size()) + " points" + (e.reversed ? " (reversed)" : "") + ">"; });

    // PyMatchCandidate struct
    py::class_<PyMatchCandidate>(m, "PyMatchCandidate", R"pbdoc(
        A candidate match location for a GPS observation point.

        Represents a potential location on the road network where a GPS point might
        actually be located, accounting for GPS error. Multiple candidates per point
        are considered during matching.
    )pbdoc")
        .def_readonly("x", &PyMatchCandidate::x, "X coordinate of the snapped candidate location")
        .def_readonly("y", &PyMatchCandidate::y, "Y coordinate of the snapped candidate location")
        .def_readonly("t", &PyMatchCandidate::t,
                      "Timestamp of the original GPS observation (if trajectory has time)")
        .def_readonly("perpendicular_distance_to_matched_geometry", &PyMatchCandidate::perpendicular_distance_to_matched_geometry,
                      "Perpendicular distance from GPS point to the matched edge geometry")
        .def_readonly("offset_from_start_of_edge", &PyMatchCandidate::offset_from_start_of_edge,
                      "Distance from the start of the candidate edge to this match location")
        .def("__repr__", [](const PyMatchCandidate &c)
             { return "<Candidate x=" + fmt::format("{:.1f}", c.x) +
                      " y=" + fmt::format("{:.1f}", c.y) +
                      " t=" + fmt::format("{:.1f}", c.t) +
                      " perpendicular_distance_to_matched_geometry=" + fmt::format("{:.1f}", c.perpendicular_distance_to_matched_geometry) +
                      " offset_from_start_of_edge=" + fmt::format("{:.1f}", c.offset_from_start_of_edge) + ">"; });

    // PyMatchSegment struct
    py::class_<PyMatchSegment>(m, "PyMatchSegment", R"pbdoc(
        A continuous matched path segment between two GPS observation points.

        Represents the matched route from one GPS point to the next, potentially
        spanning multiple road edges. Each segment contains the start/end candidates
        and the sequence of edges traversed.
    )pbdoc")
        .def_readonly("p0", &PyMatchSegment::p0,
                      "Starting candidate point of this segment")
        .def_readonly("p1", &PyMatchSegment::p1,
                      "Ending candidate point of this segment")
        .def_readonly("edges", &PyMatchSegment::edges,
                      "List of PyMatchSegmentEdge objects forming the path from p0 to p1")
        .def("__repr__", [](const PyMatchSegment &s)
             { return "<Segment from (" + fmt::format("{:.1f}", s.p0.x) + ", " + fmt::format("{:.1f}", s.p0.y) +
                      ") to (" + fmt::format("{:.1f}", s.p1.x) + ", " + fmt::format("{:.1f}", s.p1.y) +
                      ") with " + std::to_string(s.edges.size()) + " edges>"; });

    // PyMatchResult struct
    py::class_<PyMatchResult>(m, "PyMatchResult", R"pbdoc(
        Complete map matching result for a trajectory.

        Contains the matched path as a sequence of segments, error information,
        and metadata about which trajectory points were successfully matched.
    )pbdoc")
        .def_readonly("id", &PyMatchResult::id,
                      "Trajectory ID (copied from input Trajectory)")
        .def_readonly("error_code", &PyMatchResult::error_code,
                      "MatchErrorCode indicating success or failure reason")
        .def_readonly("last_connected_trajectory_point", &PyMatchResult::last_connected_trajectory_point,
                      "Index of the last GPS point that was successfully matched in a connected path")
        .def_readonly("unmatched_candidate_indices", &PyMatchResult::unmatched_candidate_indices,
                      "List of trajectory point indices that could not be matched (e.g., too far from network)")
        .def_readonly("segments", &PyMatchResult::segments,
                      "List of PyMatchSegment objects forming the complete matched path")
        .def("__repr__", [](const PyMatchResult &r)
             { return "<Match id=" + std::to_string(r.id) +
                      " error_code=" + std::to_string(static_cast<int>(r.error_code)) +
                      " last_connected_trajectory_point=" + std::to_string(r.last_connected_trajectory_point) +
                      " number unmatched candidates=" + std::to_string(r.unmatched_candidate_indices.size()) +
                      " with " + std::to_string(r.segments.size()) + " segments>"; });

    // FastMapMatch class
    py::class_<FastMapMatch>(m, "FastMapMatch", R"pbdoc(
        Fast map matching algorithm using Hidden Markov Model with UBODT optimization.

        Matches GPS trajectories to a road network by finding the most probable sequence
        of road edges, considering both emission probabilities (GPS accuracy) and transition
        probabilities (path likelihood). Uses precomputed UBODT for fast path lookups.
    )pbdoc")
        .def(py::init<const Network &, TransitionMode, std::optional<double>, std::optional<double>, const std::string &>(),
             py::arg("network"),
             py::arg("mode"),
             py::arg("max_distance_between_candidates") = std::nullopt,
             py::arg("max_time_between_candidates") = std::nullopt,
             py::arg("cache_dir") = "./ubodt_cache",
             R"pbdoc(
            Create a FastMapMatch instance with automatic UBODT management.

            Args:
                network: Road network with spatial index built (call finalize() first)
                mode: Routing mode (TransitionMode.SHORTEST for distance, FASTEST for time)
                max_distance_between_candidates: Maximum distance in meters (for SHORTEST mode)
                max_time_between_candidates: Maximum time in seconds (for FASTEST mode)
                cache_dir: Directory for caching UBODT files (default: "./ubodt_cache")

            Note:
                Only the relevant parameter is used depending on mode. This constructor automatically generates/loads UBODT from cache based
                on network hash, mode, and delta. UBODT is cached for reuse.
        )pbdoc")
        .def("pymatch_trajectory", &FastMapMatch::pymatch_trajectory,
             py::arg("trajectory"), py::arg("config"),
             R"pbdoc(
            Match a GPS trajectory to the road network.

            Applies the Hidden Markov Model map matching algorithm to find the most
            likely sequence of road edges corresponding to the input GPS points.

            Args:
                trajectory: Trajectory with GPS observations (with or without timestamps)
                config: FastMapMatchConfig with matching parameters

            Returns:
                PyMatchResult containing the matched path segments, or error information
                if matching failed

            Note:
                The config's transition_mode must match the mode used to create the
                NetworkGraph and UBODT. For FASTEST mode, ensure reference_speed is set.
        )pbdoc")
        .def("pymatch_trajectory_split", &FastMapMatch::pymatch_trajectory_split,
             py::arg("trajectory"), py::arg("config"),
             R"pbdoc(
            Match a GPS trajectory with automatic splitting on failures.

            This method performs candidate search once and reuses it when matching
            sub-trajectories, providing better performance than repeatedly calling
            pymatch_trajectory() on split segments. When the matching algorithm
            encounters failures (no candidates, disconnected layers), it automatically
            continues matching from the next viable point instead of stopping.

            Args:
                trajectory: Trajectory with GPS observations (with or without timestamps)
                config: FastMapMatchConfig with matching parameters

            Returns:
                PySplitMatchResult containing a list of sub-trajectories, each marked
                as either successfully matched (with segments) or failed (with error code)

            Example:
                Trajectory with points [0,1,2,3,4,5,6,7] where point 4 has no candidates:
                - Returns 2 sub-trajectories: [0-3] SUCCESS, [4-4] CANDIDATES_NOT_FOUND
                - If points 5-7 can be matched, adds [5-7] SUCCESS
                - Much faster than calling pymatch_trajectory() multiple times since
                  candidate lookup is done once

            Note:
                The config's transition_mode must match the mode used to create the
                NetworkGraph and UBODT. For FASTEST mode, ensure reference_speed is set.
        )pbdoc");

    // PySubTrajectory struct
    py::class_<PySubTrajectory>(m, "PySubTrajectory", R"pbdoc(
        A continuous portion of a trajectory that was matched or failed.

        Represents a successful match with segments. Failed portions are simply
        excluded from the results - only successfully matched sub-trajectories
        are returned in the PySplitMatchResult.
    )pbdoc")
        .def_readonly("start_index", &PySubTrajectory::start_index,
                      "Starting trajectory point index (inclusive)")
        .def_readonly("end_index", &PySubTrajectory::end_index,
                      "Ending trajectory point index (inclusive)")
        .def_readonly("error_code", &PySubTrajectory::error_code,
                      "MatchErrorCode: SUCCESS if matched, or failure reason (informational)")
        .def_readonly("segments", &PySubTrajectory::segments,
                      "List of PyMatchSegment objects (only populated if error_code == SUCCESS)")
        .def("__repr__", [](const PySubTrajectory &s)
             {
                 std::string status = (s.error_code == MatchErrorCode::SUCCESS) ? "SUCCESS" : "FAILED";
                 return "<SubTraj [" + std::to_string(s.start_index) + "-" +
                        std::to_string(s.end_index) + "] " + status +
                        " with " + std::to_string(s.segments.size()) + " segments>"; });

    // PySplitMatchResult struct
    py::class_<PySplitMatchResult>(m, "PySplitMatchResult", R"pbdoc(
        Result of matching with automatic trajectory splitting.

        Contains a list of sub-trajectories representing all continuous matched
        portions and failed sections of the input trajectory. Each sub-trajectory
        indicates which points it covers and whether matching succeeded or failed.
    )pbdoc")
        .def_readonly("id", &PySplitMatchResult::id,
                      "Trajectory ID (copied from input Trajectory)")
        .def_readonly("subtrajectories", &PySplitMatchResult::subtrajectories,
                      "List of PySubTrajectory objects (both successful and failed portions)")
        .def("__repr__", [](const PySplitMatchResult &r)
             {
                 int success_count = 0;
                 int failed_count = 0;
                 for (const auto &sub : r.subtrajectories) {
                     if (sub.error_code == MatchErrorCode::SUCCESS) {
                         success_count++;
                     } else {
                         failed_count++;
                     }
                 }
                 return "<SplitMatch id=" + std::to_string(r.id) +
                        " total_subs=" + std::to_string(r.subtrajectories.size()) +
                        " success=" + std::to_string(success_count) +
                        " failed=" + std::to_string(failed_count) + ">"; });

    // Trajectory struct
    py::class_<Trajectory>(m, "Trajectory", R"pbdoc(
        A GPS trajectory consisting of sequential observations.

        Trajectories can include timestamps (x, y, t) for time interpolation or be
        spatial-only (x, y). The trajectory's geometry and timestamps are used during
        map matching to find the best road path.
    )pbdoc")
        .def_readwrite("id", &Trajectory::id,
                       "Unique integer identifier for this trajectory")
        .def("__len__", [](const Trajectory &self)
             { return self.geom.get_num_points(); }, "Number of GPS observation points in the trajectory")
        .def("to_xyt_tuples", [](const Trajectory &self)
             {
            py::list tuples;
            auto data = self.to_xyt_tuples();
            for (const auto &item : data) {
                tuples.append(py::make_tuple(std::get<0>(item), std::get<1>(item), std::get<2>(item)));
            }
            return tuples; }, R"pbdoc(
            Export trajectory as a list of (x, y, t) tuples.

            Returns:
                List of tuples, each containing (x_coord, y_coord, timestamp)
        )pbdoc")
        .def("to_xy_tuples", [](const Trajectory &self)
             {
            py::list tuples;
            auto data = self.to_xy_tuples();
            for (const auto &item : data) {
                tuples.append(py::make_tuple(std::get<0>(item), std::get<1>(item)));
            }
            return tuples; }, R"pbdoc(
            Export trajectory as a list of (x, y) tuples (spatial only).

            Returns:
                List of tuples, each containing (x_coord, y_coord)
        )pbdoc")
        .def_static("from_xyt_tuples", [](int id, py::list tuples)
                    {
                std::vector<std::tuple<double, double, double>> data;
                for (auto item : tuples) {
                    if (py::isinstance<py::tuple>(item) && py::len(item) == 3) {
                        py::tuple tup = item.cast<py::tuple>();
                        double x = py::float_(tup[0]);
                        double y = py::float_(tup[1]);
                        double t = py::float_(tup[2]);
                        data.emplace_back(x, y, t);
                    } else {
                        throw std::runtime_error("Each item must be a tuple (x, y, t)");
                    }
                }
                return Trajectory::from_xyt_tuples(id, data); }, py::arg("id"), py::arg("tuples"), R"pbdoc(
            Create a Trajectory from GPS points with timestamps.

            Args:
                id: Unique integer identifier for this trajectory
                tuples: List of (x, y, t) tuples representing GPS observations with time

            Returns:
                Trajectory instance with timestamps for time interpolation
        )pbdoc")
        .def_static("from_xy_tuples", [](int id, py::list tuples)
                    {
                std::vector<std::tuple<double, double>> data;
                for (auto item : tuples) {
                    if (py::isinstance<py::tuple>(item) && py::len(item) == 2) {
                        py::tuple tup = item.cast<py::tuple>();
                        double x = py::float_(tup[0]);
                        double y = py::float_(tup[1]);
                        data.emplace_back(x, y);
                    } else {
                        throw std::runtime_error("Each item must be a tuple (x, y)");
                    }
                }
                return Trajectory::from_xy_tuples(id, data); }, py::arg("id"), py::arg("tuples"), R"pbdoc(
            Create a Trajectory from GPS points without timestamps (spatial-only).

            Args:
                id: Unique integer identifier for this trajectory
                tuples: List of (x, y) tuples representing GPS observation locations

            Returns:
                Trajectory instance without time information
        )pbdoc");
}
