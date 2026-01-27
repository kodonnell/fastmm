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
    py::class_<Point>(m, "Point")
        .def(py::init<double, double>(), py::arg("x"), py::arg("y"), "Create a Point with x, y coordinates")
        .def("get_x", [](const Point &p)
             { return boost::geometry::get<0>(p); }, "Get x coordinate")
        .def("get_y", [](const Point &p)
             { return boost::geometry::get<1>(p); }, "Get y coordinate")
        .def("__repr__", [](const Point &p)
             { return "<Point x=" + std::to_string(boost::geometry::get<0>(p)) +
                      " y=" + std::to_string(boost::geometry::get<1>(p)) + ">"; });

    // LineString class
    py::class_<LineString>(m, "LineString")
        .def(py::init<>(), "Create an empty LineString")
        .def("add_point", py::overload_cast<double, double>(&LineString::add_point),
             py::arg("x"), py::arg("y"), "Add a point to the linestring")
        .def("get_num_points", &LineString::get_num_points, "Get the number of points in the linestring")
        .def("get_x", &LineString::get_x, py::arg("i"), "Get x coordinate of i-th point")
        .def("get_y", &LineString::get_y, py::arg("i"), "Get y coordinate of i-th point")
        .def("get_length", &LineString::get_length, "Get the length of the linestring")
        .def("export_wkt", &LineString::export_wkt, py::arg("precision") = 8, "Export as WKT string")
        .def("export_json", &LineString::export_json, "Export as GeoJSON string")
        .def_static("from_wkt", &wkt2linestring, py::arg("wkt"), "Create a LineString from WKT")
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
    py::class_<Network>(m, "Network")
        .def(py::init<>(), "Create an empty network. Use add_edge() to populate it, then call build_rtree_index().")
        .def("add_edge", &Network::add_edge,
             py::arg("edge_id"),
             py::arg("source"),
             py::arg("target"),
             py::arg("geom"),
             py::arg("speed") = std::nullopt,
             "Add an edge to the network. Call build_rtree_index() after adding all edges.")
        .def("build_rtree_index", &Network::build_rtree_index,
             "Build the spatial index. Must be called after adding all edges and before performing spatial queries.")
        .def("get_edge_count", &Network::get_edge_count)
        .def("get_node_count", &Network::get_node_count);

    // NetworkGraph class
    py::class_<NetworkGraph>(m, "NetworkGraph")
        .def(py::init<const Network &, TransitionMode>(), 
             py::arg("network"), 
             py::arg("mode") = TransitionMode::SHORTEST,
             "Create a NetworkGraph from a Network with specified routing mode");

    // UBODTGenAlgorithm class
    py::class_<UBODTGenAlgorithm>(m, "UBODTGenAlgorithm")
        .def(py::init<const Network &, const NetworkGraph &>(), py::arg("network"), py::arg("graph"))
        .def("generate_ubodt", &UBODTGenAlgorithm::generate_ubodt,
             py::arg("filename"), py::arg("delta"));

    // UBODT class
    py::class_<UBODT, std::shared_ptr<UBODT>>(m, "UBODT")
        .def_static("read_ubodt_binary", &UBODT::read_ubodt_binary,
                    py::arg("filename"), py::arg("multiplier"));

    // FastMapMatchConfig class
    py::class_<FastMapMatchConfig>(m, "FastMapMatchConfig")
        .def(py::init<int, double, double, double, TransitionMode, std::optional<double>>(),
             py::arg("k") = 8,
             py::arg("candidate_search_radius") = 50,
             py::arg("gps_error") = 50,
             py::arg("reverse_tolerance") = 0.0,
             py::arg("transition_mode") = TransitionMode::SHORTEST,
             py::arg("reference_speed") = std::nullopt,
             "Create a FastMapMatchConfig. For FASTEST mode, reference_speed must be provided.")
        .def_readwrite("k", &FastMapMatchConfig::k)
        .def_readwrite("candidate_search_radius", &FastMapMatchConfig::candidate_search_radius)
        .def_readwrite("gps_error", &FastMapMatchConfig::gps_error)
        .def_readwrite("reverse_tolerance", &FastMapMatchConfig::reverse_tolerance)
        .def_readwrite("transition_mode", &FastMapMatchConfig::transition_mode)
        .def_readwrite("reference_speed", &FastMapMatchConfig::reference_speed);

    // PyMatchPoint struct
    py::class_<PyMatchPoint>(m, "PyMatchPoint")
        .def_readonly("x", &PyMatchPoint::x)
        .def_readonly("y", &PyMatchPoint::y)
        .def_readonly("edge_offset", &PyMatchPoint::edge_offset)
        .def_readonly("cumulative_distance", &PyMatchPoint::cumulative_distance)
        .def("__repr__", [](const PyMatchPoint &p)
             { return "<P x=" + fmt::format("{:.1f}", p.x) +
                      " y=" + fmt::format("{:.1f}", p.y) +
                      " edge_offset=" + fmt::format("{:.1f}", p.edge_offset) +
                      " cumulative_distance=" + fmt::format("{:.1f}", p.cumulative_distance) + ">"; });

    // PyMatchSegmentEdge struct
    py::class_<PyMatchSegmentEdge>(m, "PyMatchSegmentEdge")
        .def_readonly("edge_id", &PyMatchSegmentEdge::edge_id)
        .def_readonly("points", &PyMatchSegmentEdge::points)
        .def("__repr__", [](const PyMatchSegmentEdge &e)
             { return "<Edge edge_id=" + std::to_string(e.edge_id) + " with " + std::to_string(e.points.size()) + " points>"; });

    // PyMatchCandidate struct
    py::class_<PyMatchCandidate>(m, "PyMatchCandidate")
        .def_readonly("x", &PyMatchCandidate::x)
        .def_readonly("y", &PyMatchCandidate::y)
        .def_readonly("t", &PyMatchCandidate::t)
        .def_readonly("perpendicular_distance_to_matched_geometry", &PyMatchCandidate::perpendicular_distance_to_matched_geometry)
        .def_readonly("offset_from_start_of_edge", &PyMatchCandidate::offset_from_start_of_edge)
        .def("__repr__", [](const PyMatchCandidate &c)
             { return "<Candidate x=" + fmt::format("{:.1f}", c.x) +
                      " y=" + fmt::format("{:.1f}", c.y) +
                      " t=" + fmt::format("{:.1f}", c.t) +
                      " perpendicular_distance_to_matched_geometry=" + fmt::format("{:.1f}", c.perpendicular_distance_to_matched_geometry) +
                      " offset_from_start_of_edge=" + fmt::format("{:.1f}", c.offset_from_start_of_edge) + ">"; });

    // PyMatchSegment struct
    py::class_<PyMatchSegment>(m, "PyMatchSegment")
        .def_readonly("p0", &PyMatchSegment::p0)
        .def_readonly("p1", &PyMatchSegment::p1)
        .def_readonly("edges", &PyMatchSegment::edges)
        .def("__repr__", [](const PyMatchSegment &s)
             { return "<Segment from (" + fmt::format("{:.1f}", s.p0.x) + ", " + fmt::format("{:.1f}", s.p0.y) +
                      ") to (" + fmt::format("{:.1f}", s.p1.x) + ", " + fmt::format("{:.1f}", s.p1.y) +
                      ") with " + std::to_string(s.edges.size()) + " edges>"; });

    // PyMatchResult struct
    py::class_<PyMatchResult>(m, "PyMatchResult")
        .def_readonly("id", &PyMatchResult::id)
        .def_readonly("error_code", &PyMatchResult::error_code)
        .def_readonly("last_connected_trajectory_point", &PyMatchResult::last_connected_trajectory_point)
        .def_readonly("unmatched_candidate_indices", &PyMatchResult::unmatched_candidate_indices)
        .def_readonly("segments", &PyMatchResult::segments)
        .def("__repr__", [](const PyMatchResult &r)
             { return "<Match id=" + std::to_string(r.id) +
                      " error_code=" + std::to_string(static_cast<int>(r.error_code)) +
                      " last_connected_trajectory_point=" + std::to_string(r.last_connected_trajectory_point) +
                      " number unmatched candidates=" + std::to_string(r.unmatched_candidate_indices.size()) +
                      " with " + std::to_string(r.segments.size()) + " segments>"; });

    // FastMapMatch class
    py::class_<FastMapMatch>(m, "FastMapMatch")
        .def(py::init<const Network &, const NetworkGraph &, std::shared_ptr<UBODT>>(),
             py::arg("network"), py::arg("graph"), py::arg("ubodt"))
        .def("pymatch_trajectory", &FastMapMatch::pymatch_trajectory,
             py::arg("trajectory"), py::arg("config"));

    // Trajectory struct
    py::class_<Trajectory>(m, "Trajectory")
        .def_readwrite("id", &Trajectory::id)
        .def("__len__", [](const Trajectory &self)
             { return self.geom.get_num_points(); })
        .def("to_xyt_tuples", [](const Trajectory &self)
             {
            py::list tuples;
            auto data = self.to_xyt_tuples();
            for (const auto &item : data) {
                tuples.append(py::make_tuple(std::get<0>(item), std::get<1>(item), std::get<2>(item)));
            }
            return tuples; }, R"pbdoc(Return the trajectory as a list of (x, y, t) tuples)pbdoc")
        .def("__len__", [](const Trajectory &self)
             { return self.size(); })
        .def("to_xy_tuples", [](const Trajectory &self)
             {
            py::list tuples;
            auto data = self.to_xy_tuples();
            for (const auto &item : data) {
                tuples.append(py::make_tuple(std::get<0>(item), std::get<1>(item)));
            }
            return tuples; }, R"pbdoc(Return the trajectory as a list of (x, y) tuples)pbdoc")
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
                return Trajectory::from_xyt_tuples(id, data); }, py::arg("id"), py::arg("tuples"), R"pbdoc(Create a Trajectory from a list of (x, y, t) tuples)pbdoc")
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
                return Trajectory::from_xy_tuples(id, data); }, py::arg("id"), py::arg("tuples"), R"pbdoc(Create a Trajectory from a list of (x, y) tuples)pbdoc");
}
