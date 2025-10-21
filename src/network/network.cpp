#include "network/network.hpp"
#include "util/debug.hpp"
#include "util/util.hpp"
#include "algorithm/geom_algorithm.hpp"

#include <math.h>    // Calulating probability
#include <algorithm> // Partial sort copy
#include <stdexcept>

// Data structures for Rtree
#include <boost/geometry/index/rtree.hpp>
#include <boost/iterator/function_output_iterator.hpp>

#include <boost/format.hpp>

using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::MM;
using namespace FASTMM::NETWORK;

bool Network::candidate_compare(const Candidate &a, const Candidate &b)
{
  if (a.dist != b.dist)
  {
    return a.dist < b.dist;
  }
  else
  {
    return a.edge->index < b.edge->index;
  }
}

Network::Network() : num_vertices(0)
{
  SPDLOG_INFO("Created empty network");
}

void Network::add_edge(EdgeID edge_id, NodeID source, NodeID target,
                       const FASTMM::CORE::LineString &geom)
{
  NodeIndex s_idx, t_idx;
  if (node_map.find(source) == node_map.end())
  {
    s_idx = node_id_vec.size();
    node_id_vec.push_back(source);
    node_map.insert({source, s_idx});
    vertex_points.push_back(geom.get_point(0));
  }
  else
  {
    s_idx = node_map[source];
  }
  if (node_map.find(target) == node_map.end())
  {
    t_idx = node_id_vec.size();
    node_id_vec.push_back(target);
    node_map.insert({target, t_idx});
    int npoints = geom.get_num_points();
    vertex_points.push_back(geom.get_point(npoints - 1));
  }
  else
  {
    t_idx = node_map[target];
  }
  EdgeIndex index = edges.size();
  edges.push_back({index, edge_id, s_idx, t_idx, geom.get_length(), geom});
  edge_map.insert({edge_id, index});
  num_vertices = node_id_vec.size();
};

int Network::get_node_count() const
{
  return node_id_vec.size();
}

int Network::get_edge_count() const
{
  return edges.size();
}

// Get the edge vector
const std::vector<Edge> &Network::get_edges() const
{
  return edges;
}

const Edge &Network::get_edge(EdgeID id) const
{
  return edges[get_edge_index(id)];
};

const Edge &Network::get_edge(EdgeIndex index) const
{
  return edges[index];
};

// Get the ID attribute of an edge according to its index
EdgeID Network::get_edge_id(EdgeIndex index) const
{
  return index < edges.size() ? edges[index].id : -1;
}

EdgeIndex Network::get_edge_index(EdgeID id) const
{
  return edge_map.at(id);
}

NodeID Network::get_node_id(NodeIndex index) const
{
  return index < num_vertices ? node_id_vec[index] : -1;
}

NodeIndex Network::get_node_index(NodeID id) const
{
  return node_map.at(id);
}

Point Network::get_node_geom_from_idx(NodeIndex index) const
{
  return vertex_points[index];
}

// Construct a Rtree using the vector of edges
void Network::build_rtree_index()
{
  // Build an rtree for candidate search
  SPDLOG_DEBUG("Create boost rtree");
  // create some Items
  for (std::size_t i = 0; i < edges.size(); ++i)
  {
    // create a boost_box
    Edge *edge = &edges[i];
    double x1, y1, x2, y2;
    ALGORITHM::boundingbox_geometry(edge->geom, &x1, &y1, &x2, &y2);
    boost_box b(Point(x1, y1), Point(x2, y2));
    rtree.insert(std::make_pair(b, edge));
  }
  SPDLOG_DEBUG("Create boost rtree done");
}

TrajectoryCandidates Network::search_tr_cs_knn(Trajectory &trajectory, std::size_t k, double radius) const
{
  return search_tr_cs_knn(trajectory.geom, k, radius);
}

TrajectoryCandidates Network::search_tr_cs_knn(const LineString &geom, std::size_t k, double radius) const
{
  int NumberPoints = geom.get_num_points();
  TrajectoryCandidates tr_cs(NumberPoints);
  unsigned int current_candidate_index = num_vertices;
  for (int i = 0; i < NumberPoints; ++i)
  {
    // SPDLOG_DEBUG("Search candidates for point index {}",i);
    // Construct a bounding boost_box
    double px = geom.get_x(i);
    double py = geom.get_y(i);
    PointCandidates pcs;
    boost_box b(Point(geom.get_x(i) - radius, geom.get_y(i) - radius), Point(geom.get_x(i) + radius, geom.get_y(i) + radius));
    std::vector<Item> temp;
    // Rtree can only detect intersect with a the bounding box of
    // the geometry stored.
    rtree.query(boost::geometry::index::intersects(b), std::back_inserter(temp));
    int Nitems = temp.size();
    for (unsigned int j = 0; j < Nitems; ++j)
    {
      // Check for detailed intersection
      Edge *edge = temp[j].second;
      double offset;
      double dist;
      double closest_x, closest_y;
      ALGORITHM::linear_referencing(px, py, edge->geom, &dist, &offset, &closest_x, &closest_y);
      if (dist <= radius)
      {
        // index, offset, dist, edge, pseudo id, point
        Candidate c = {0, offset, dist, edge, Point(closest_x, closest_y)};
        pcs.push_back(c);
      }
    }
    SPDLOG_DEBUG("Candidate count point {}: {} (filter to k)", i, pcs.size());
    if (pcs.empty())
    {
      SPDLOG_DEBUG("Candidate not found for point {}: {} {}", i, px, py);
    }
    else
    {
      // KNN part
      if (pcs.size() <= k)
      {
        tr_cs[i] = pcs;
      }
      else
      {
        tr_cs[i] = PointCandidates(k);
        std::partial_sort_copy(
            pcs.begin(), pcs.end(),
            tr_cs[i].begin(), tr_cs[i].end(),
            candidate_compare);
      }
      for (int m = 0; m < tr_cs[i].size(); ++m)
      {
        tr_cs[i][m].index = current_candidate_index + m;
      }
      current_candidate_index += tr_cs[i].size();
      // SPDLOG_TRACE("current_candidate_index {}",current_candidate_index);
    }
  }
  return tr_cs;
}

const LineString &Network::get_edge_geom(EdgeID edge_id) const
{
  return edges[get_edge_index(edge_id)].geom;
}

LineString Network::complete_path_to_geometry(
    const LineString &traj, const CompletePath &complete_path) const
{
  // if (complete_path->empty()) return nullptr;
  LineString line;
  if (complete_path.empty())
    return line;
  int Npts = traj.get_num_points();
  int NCsegs = complete_path.size();
  if (NCsegs == 1)
  {
    double dist;
    double firstoffset;
    double lastoffset;
    const LineString &firstseg = get_edge_geom(complete_path[0]);
    ALGORITHM::linear_referencing(traj.get_x(0), traj.get_y(0), firstseg,
                                  &dist, &firstoffset);
    ALGORITHM::linear_referencing(traj.get_x(Npts - 1), traj.get_y(Npts - 1),
                                  firstseg, &dist, &lastoffset);
    LineString firstlineseg = ALGORITHM::cutoffseg_unique(firstseg, firstoffset,
                                                          lastoffset);
    append_segs_to_line(&line, firstlineseg, 0);
  }
  else
  {
    const LineString &firstseg = get_edge_geom(complete_path[0]);
    const LineString &lastseg = get_edge_geom(complete_path[NCsegs - 1]);
    double dist;
    double firstoffset;
    double lastoffset;
    ALGORITHM::linear_referencing(traj.get_x(0), traj.get_y(0), firstseg,
                                  &dist, &firstoffset);
    ALGORITHM::linear_referencing(traj.get_x(Npts - 1), traj.get_y(Npts - 1),
                                  lastseg, &dist, &lastoffset);
    LineString firstlineseg = ALGORITHM::cutoffseg(firstseg, firstoffset, 0);
    LineString lastlineseg = ALGORITHM::cutoffseg(lastseg, lastoffset, 1);
    append_segs_to_line(&line, firstlineseg, 0);
    if (NCsegs > 2)
    {
      for (int i = 1; i < NCsegs - 1; ++i)
      {
        const LineString &middleseg = get_edge_geom(complete_path[i]);
        append_segs_to_line(&line, middleseg, 1);
      }
    }
    append_segs_to_line(&line, lastlineseg, 1);
  }
  return line;
}

const std::vector<Point> &Network::get_vertex_points() const
{
  return vertex_points;
}

const Point &Network::get_vertex_point(NodeIndex u) const
{
  return vertex_points[u];
}

LineString Network::route2geometry(const std::vector<EdgeID> &path) const
{
  LineString line;
  if (path.empty())
    return line;
  // if (complete_path->empty()) return nullptr;
  int NCsegs = path.size();
  for (int i = 0; i < NCsegs; ++i)
  {
    EdgeIndex e = get_edge_index(path[i]);
    const LineString &seg = edges[e].geom;
    if (i == 0)
    {
      append_segs_to_line(&line, seg, 0);
    }
    else
    {
      append_segs_to_line(&line, seg, 1);
    }
  }
  return line;
}

LineString Network::route2geometry(const std::vector<EdgeIndex> &path) const
{
  LineString line;
  if (path.empty())
    return line;
  // if (complete_path->empty()) return nullptr;
  int NCsegs = path.size();
  for (int i = 0; i < NCsegs; ++i)
  {
    const LineString &seg = edges[path[i]].geom;
    if (i == 0)
    {
      append_segs_to_line(&line, seg, 0);
    }
    else
    {
      append_segs_to_line(&line, seg, 1);
    }
  }
  return line;
}

void Network::append_segs_to_line(LineString *line,
                                  const LineString &segs, int offset)
{
  int Npoints = segs.get_num_points();
  for (int i = 0; i < Npoints; ++i)
  {
    if (i >= offset)
    {
      line->add_point(segs.get_x(i), segs.get_y(i));
    }
  }
}
