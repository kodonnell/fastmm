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

    // Point class
    py::class_<Point>(m, "Point", R"pbdoc(
        A 2D point representing a geographic location.

        Points are used to construct LineStrings and represent trajectory locations.
        Coordinates should be in the same coordinate reference system (CRS) as your
        network edges, typically a projected coordinate system (e.g., meters).
    )pbdoc")
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"), R"pbdoc(
            Create a Point with x, y coordinates.

            Args:
                x: X coordinate (typically easting or longitude)
                y: Y coordinate (typically northing or latitude)
        )pbdoc")
        .def("get_x", [](const Point &p)
             { return boost::geometry::get<0>(p); }, R"pbdoc(
            Get the x coordinate of the point.

            Returns:
                float: The x coordinate
        )pbdoc")
        .def("get_y", [](const Point &p)
             { return boost::geometry::get<1>(p); }, R"pbdoc(
            Get the y coordinate of the point.

            Returns:
                float: The y coordinate
        )pbdoc")
        .def("__repr__", [](const Point &p)
             { return "<Point x=" + std::to_string(boost::geometry::get<0>(p)) +
                      " y=" + std::to_string(boost::geometry::get<1>(p)) + ">"; });

    // LineString class
    py::class_<LineString>(m, "LineString", R"pbdoc(
        A linestring geometry representing a road edge or path.

        LineStrings are sequences of connected points that define the geometric shape
        of road network edges. They must contain at least 2 points and should be in
        the same coordinate reference system as your network.
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Create an empty LineString.

            Use add_point() to populate the linestring with points.
        )pbdoc")
        .def("add_point", py::overload_cast<double, double>(&LineString::add_point),
             py::arg("x"), py::arg("y"), R"pbdoc(
            Add a point to the linestring.

            Points are added in sequence to form the line geometry.

            Args:
                x: X coordinate of the point
                y: Y coordinate of the point
        )pbdoc")
        .def("get_num_points", &LineString::get_num_points, R"pbdoc(
            Get the number of points in the linestring.

            Returns:
                int: Number of points
        )pbdoc")
        .def("get_x", &LineString::get_x, py::arg("i"), R"pbdoc(
            Get x coordinate of the i-th point.

            Args:
                i: Zero-based index of the point

            Returns:
                float: X coordinate of the point
        )pbdoc")
        .def("get_y", &LineString::get_y, py::arg("i"), R"pbdoc(
            Get y coordinate of the i-th point.

            Args:
                i: Zero-based index of the point

            Returns:
                float: Y coordinate of the point
        )pbdoc")
        .def("get_length", &LineString::get_length, R"pbdoc(
            Get the Euclidean length of the linestring.

            Returns:
                float: Total length in coordinate units
        )pbdoc")
        .def("export_wkt", &LineString::export_wkt, py::arg("precision") = 8, R"pbdoc(
            Export the linestring as a Well-Known Text (WKT) string.

            Args:
                precision: Number of decimal places for coordinates (default: 8)

            Returns:
                str: WKT representation (e.g., 'LINESTRING(0 0, 1 1)')
        )pbdoc")
        .def("export_json", &LineString::export_json, R"pbdoc(
            Export the linestring as a GeoJSON string.

            Returns:
                str: GeoJSON LineString geometry
        )pbdoc")
        .def_static("from_wkt", &wkt2linestring, py::arg("wkt"), R"pbdoc(
            Create a LineString from a Well-Known Text (WKT) string.

            Args:
                wkt: WKT string representing a linestring (e.g., 'LINESTRING(0 0, 1 1)')

            Returns:
                LineString: Parsed linestring geometry
        )pbdoc")
        .def("__repr__", [](const LineString &l)
             { return "<LineString with " + std::to_string(l.get_num_points()) + " points, length=" +
                      std::to_string(l.get_length()) + ">"; });

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
            >>> network.add_edge(1, source=10, target=20, geom=linestring, speed=50.0)
            >>> network.build_rtree_index()
    )pbdoc")
        .def(py::init<>(), R"pbdoc(
            Create an empty network.

            Use add_edge() to populate the network with road segments, then call
            build_rtree_index() to prepare it for map matching.
        )pbdoc")
        .def("add_edge", &Network::add_edge,
             py::arg("edge_id"),
             py::arg("source"),
             py::arg("target"),
             py::arg("geom"),
             py::arg("speed") = std::nullopt,
             R"pbdoc(
            Add a directed edge (road segment) to the network.

            Each edge must have a unique ID and connects two nodes. The geometry defines
            the spatial path of the edge. Speed is required for FASTEST routing mode.

            Args:
                edge_id: Unique integer identifier for this edge
                source: Node ID where the edge starts
                target: Node ID where the edge ends
                geom: LineString defining the edge's spatial geometry
                speed: Optional speed value (distance units per time unit).
                       Required if using TransitionMode.FASTEST routing.

            Note:
                Call build_rtree_index() after adding all edges.
        )pbdoc")
        .def("build_rtree_index", &Network::build_rtree_index,
             R"pbdoc(
            Build the spatial R-tree index for efficient candidate edge lookup.

            This MUST be called after adding all edges and before creating a NetworkGraph
            or performing any map matching operations. The index enables fast spatial
            queries to find nearby road segments for GPS points.

            Raises:
                RuntimeError: If called on an empty network
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
                network: Network with edges (must have build_rtree_index() called)
                mode: Routing mode - SHORTEST (distance-based) or FASTEST (time-based).
                      Default is SHORTEST.

            Raises:
                RuntimeError: If mode is FASTEST and any edge lacks a speed value

            Note:
                Generate separate UBODT files for each routing mode, as the shortest
                paths differ between distance-based and time-based routing.
        )pbdoc");

    // UBODTGenAlgorithm class
    py::class_<UBODTGenAlgorithm>(m, "UBODTGenAlgorithm", R"pbdoc(
        Algorithm for generating Upper Bounded Origin-Destination Table (UBODT).

        UBODT is a precomputed routing table that stores shortest paths between edges
        within a maximum distance/time threshold. This dramatically speeds up map matching
        by avoiding repeated Dijkstra searches during matching.
    )pbdoc")
        .def(py::init<const Network &, const NetworkGraph &>(), py::arg("network"), py::arg("graph"),
             R"pbdoc(
            Create a UBODT generator for the given network and graph.

            Args:
                network: Network containing the road edges
                graph: NetworkGraph with routing configuration (SHORTEST or FASTEST)
        )pbdoc")
        .def("generate_ubodt", &UBODTGenAlgorithm::generate_ubodt,
             py::arg("filename"), py::arg("delta"),
             R"pbdoc(
            Generate and save UBODT to a binary file.

            This computes shortest paths from each edge to all reachable edges within
            the delta threshold, then serializes the routing table to disk.

            Args:
                filename: Output file path for the binary UBODT file
                delta: Maximum routing cost threshold (distance for SHORTEST mode,
                       time for FASTEST mode). Only paths with cost ≤ delta are stored.
                       Larger values improve matching quality but increase file size
                       and generation time.

            Note:
                Generation can take significant time for large networks. The delta value
                should be at least 2-3x your maximum expected distance between consecutive
                GPS points to allow for routing flexibility.
        )pbdoc");

    // UBODT class
    py::class_<UBODT, std::shared_ptr<UBODT>>(m, "UBODT", R"pbdoc(
        Upper Bounded Origin-Destination Table for fast path lookups.

        UBODT stores precomputed shortest paths between edges, enabling O(1) lookups
        during map matching instead of running Dijkstra's algorithm repeatedly.
    )pbdoc")
        .def_static("read_ubodt_binary", &UBODT::read_ubodt_binary,
                    py::arg("filename"), py::arg("multiplier"),
                    R"pbdoc(
            Load a UBODT from a binary file.

            Args:
                filename: Path to the binary UBODT file (generated by UBODTGenAlgorithm)
                multiplier: Scaling factor for internal hash table size. Higher values
                           (e.g., 50000) reduce hash collisions and improve lookup speed
                           but use more memory. Typical values: 10000-100000.

            Returns:
                Shared pointer to loaded UBODT instance

            Note:
                The UBODT must match the network and routing mode (SHORTEST/FASTEST)
                used for map matching. Using mismatched UBODTs will produce incorrect results.
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
        .def("__repr__", [](const PyMatchSegmentEdge &e)
             { return "<Edge edge_id=" + std::to_string(e.edge_id) + " with " + std::to_string(e.points.size()) + " points>"; });

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
        .def(py::init<const Network &, const NetworkGraph &, std::shared_ptr<UBODT>>(),
             py::arg("network"), py::arg("graph"), py::arg("ubodt"),
             R"pbdoc(
            Create a FastMapMatch instance.

            Args:
                network: Road network with spatial index built
                graph: NetworkGraph for routing (must match UBODT's routing mode)
                ubodt: Precomputed routing table (must match network and routing mode)

            Note:
                All three components must be consistent: use the same Network instance
                and ensure the NetworkGraph and UBODT were created with the same
                TransitionMode (SHORTEST or FASTEST).
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
        )pbdoc");

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
