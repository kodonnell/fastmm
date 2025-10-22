//
// Created by Can Yang on 2020/3/22.
//

#include "mm/fmm/fmm_algorithm.hpp"
#include "algorithm/geom_algorithm.hpp"
#include "util/util.hpp"
#include "util/debug.hpp"

using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::NETWORK;
using namespace FASTMM::MM;

FastMapMatchConfig::FastMapMatchConfig(int k_arg,
                                       double candidate_search_radius,
                                       double gps_error,
                                       double reverse_tolerance) : k(k_arg),
                                                                   candidate_search_radius(candidate_search_radius),
                                                                   gps_error(gps_error),
                                                                   reverse_tolerance(reverse_tolerance) {
                                                                   };

MatchResult FastMapMatch::match_trajectory(const Trajectory &trajectory, const FastMapMatchConfig &config)
{
  SPDLOG_DEBUG("Count of points in trajectory {}", trajectory.geom.get_num_points());
  SPDLOG_DEBUG("Search candidates");
  TrajectoryCandidates tc = network_.search_tr_cs_knn(trajectory.geom, config.k, config.candidate_search_radius);
  SPDLOG_DEBUG("Trajectory candidate {}", tc);
  MatchResult result = MatchResult{};
  result.id = trajectory.id;
  result.error_code = MatchErrorCode::UNKNOWN_ERROR;
  std::vector<int> unmatched_indices;
  for (int i = 0; i < tc.size(); ++i)
  {
    if (tc[i].empty())
    {
      unmatched_indices.push_back(i);
    }
  }

  if (!unmatched_indices.empty())
  {
    SPDLOG_DEBUG("No candidates found for trajectory {} at points {}", trajectory.id, unmatched_indices);
    result.error_code = MatchErrorCode::CANDIDATES_NOT_FOUND;
    result.unmatched_candidate_indices = unmatched_indices;
    return result;
  }

  SPDLOG_DEBUG("Generate transition graph");
  TransitionGraph tg(tc, config.gps_error);
  SPDLOG_DEBUG("Update cost in transition graph");
  // The network will be used internally to update transition graph
  bool all_connected = false;
  int last_connected = update_tg(&tg, trajectory, &all_connected, config.reverse_tolerance);
  if (!all_connected)
  {
    SPDLOG_DEBUG("Traj {} unmatched at trajectory point {}", trajectory.id, last_connected);
    result.last_connected_trajectory_point = last_connected;
    result.error_code = MatchErrorCode::DISCONNECTED_LAYERS;
    return result;
  }
  SPDLOG_DEBUG("Optimal path inference");
  TGOpath tg_opath = tg.backtrack();
  SPDLOG_DEBUG("Optimal path size {}", tg_opath.size());
  MatchedCandidatePath matched_candidate_path(tg_opath.size());
  std::transform(tg_opath.begin(), tg_opath.end(), matched_candidate_path.begin(), [](const TGNode *a)
                 { return MatchedCandidate{
                       *(a->c), a->ep, a->tp, a->shortest_path_distance}; });
  OptimalPath optimal_path(tg_opath.size());
  std::transform(tg_opath.begin(), tg_opath.end(), optimal_path.begin(), [](const TGNode *a)
                 { return a->c->edge->id; });
  std::vector<int> indices;
  const std::vector<Edge> &edges = network_.get_edges();
  CompletePath complete_path = ubodt_->construct_complete_path(trajectory.id, tg_opath, edges, &indices, config.reverse_tolerance);
  SPDLOG_DEBUG("Opath is {}", optimal_path);
  SPDLOG_DEBUG("Indices is {}", indices);
  SPDLOG_DEBUG("Complete path is {}", complete_path);
  LineString matched_geometry = network_.complete_path_to_geometry(trajectory.geom, complete_path);

  result.error_code = MatchErrorCode::SUCCESS;
  result.opt_candidate_path = matched_candidate_path;
  result.optimal_path = optimal_path;
  result.complete_path = complete_path;
  result.indices = indices;
  result.matched_geometry = matched_geometry;
  return result;
}

PyMatchResult FastMapMatch::pymatch_trajectory(const CORE::Trajectory &trajectory,
                                               const FastMapMatchConfig &config)
{
  MatchResult result = match_trajectory(trajectory, config);
  PyMatchResult output;
  output.id = result.id;
  output.error_code = result.error_code;
  output.last_connected_trajectory_point = result.last_connected_trajectory_point;
  output.unmatched_candidate_indices = result.unmatched_candidate_indices;
  output.segments = std::vector<PyMatchSegment>{};
  FASTMM::MM::CompletePath edge_ids = result.complete_path;

  if (result.error_code != MatchErrorCode::SUCCESS)
  {
    return output;
  }

  // For each of the candidate points, get the segment from p0 to p1, and the edges in between:
  double cumulative_distance = 0;
  for (int i = 1; i < result.opt_candidate_path.size(); ++i)
  {
    const MatchedCandidate &mc0 = result.opt_candidate_path[i - 1];
    const MatchedCandidate &mc1 = result.opt_candidate_path[i];
    // Get the first edge, and add the segment from the match point to the end of the edge:
    const int start_edge_index = result.indices[i - 1];
    const int end_edge_index = result.indices[i];
    if (start_edge_index < 0 || start_edge_index >= edge_ids.size())
    {
      SPDLOG_WARN("Start edge index {} out of bounds [0, {})", start_edge_index, edge_ids.size());
      output.error_code = MatchErrorCode::INDEX_OUT_OF_BOUNDS;
      return output;
    }
    if (end_edge_index < 0 || end_edge_index >= edge_ids.size())
    {
      SPDLOG_WARN("End edge index {} out of bounds [0, {})", end_edge_index, edge_ids.size());
      output.error_code = MatchErrorCode::INDEX_OUT_OF_BOUNDS_END;
      return output;
    }

    const PyMatchCandidate start_candidate = {
        boost::geometry::get<0>(mc0.c.point),
        boost::geometry::get<1>(mc0.c.point),
        trajectory.timestamps.empty() ? 0.0 : trajectory.timestamps[i - 1],
        mc0.c.dist,
        mc0.c.offset};
    const PyMatchCandidate end_candidate = {
        boost::geometry::get<0>(mc1.c.point),
        boost::geometry::get<1>(mc1.c.point),
        trajectory.timestamps.empty() ? 0.0 : trajectory.timestamps[i],
        mc1.c.dist,
        mc1.c.offset};
    double start_offset, end_offset, start_distance;
    FASTMM::CORE::LineString line;
    std::vector<PyMatchPoint> points;
    PyMatchSegment segment = {start_candidate, end_candidate, {}};
    std::vector<double> distances;
    EdgeID edge_id;
    if (start_edge_index == end_edge_index)
    {
      // OK, this segment is just one edge:
      edge_id = result.complete_path[start_edge_index];
      const Edge &e0 = network_.get_edge(edge_id);
      start_offset = mc0.c.offset;
      end_offset = mc1.c.offset;
      line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, end_offset);
      points.clear();
      distances = ALGORITHM::calculate_linestring_euclidean_distances(line);
      start_distance = cumulative_distance;
      for (int j = 0; j < line.get_num_points(); ++j)
      {
        if (j > 0)
        {
          cumulative_distance += distances[j - 1];
        }
        points.push_back({line.get_x(j), line.get_y(j), cumulative_distance - start_distance, cumulative_distance});
      }
      const PyMatchSegmentEdge segmentEdge = {edge_id, points};
      segment.edges.push_back(segmentEdge);
    }
    else
    {
      // OK, multiple segments.

      // First, the segment from the start point to the end of the first edge:
      edge_id = result.complete_path[start_edge_index];
      const Edge &e0 = network_.get_edge(edge_id);
      start_offset = mc0.c.offset;
      line = ALGORITHM::cutoffseg_unique(e0.geom, start_offset, e0.length);
      points.clear();
      distances = ALGORITHM::calculate_linestring_euclidean_distances(line);
      start_distance = cumulative_distance;
      for (int j = 0; j < line.get_num_points(); ++j)
      {
        if (j > 0)
        {
          cumulative_distance += distances[j - 1];
        }
        points.push_back({line.get_x(j), line.get_y(j), cumulative_distance - start_distance, cumulative_distance});
      }
      segment.edges.push_back({edge_id, points});

      // Next, all the full edges in between:
      for (int j = start_edge_index + 1; j < end_edge_index; ++j)
      {
        edge_id = result.complete_path[j];
        line = network_.get_edge(edge_id).geom;
        distances = ALGORITHM::calculate_linestring_euclidean_distances(line);
        points.clear();
        start_distance = cumulative_distance;
        for (int k = 0; k < line.get_num_points(); ++k)
        {
          if (k > 0)
          {
            cumulative_distance += distances[k - 1];
          }
          points.push_back({line.get_x(k), line.get_y(k), cumulative_distance - start_distance, cumulative_distance});
        }
        segment.edges.push_back({edge_id, points});
      }
      // Finally, the segment from the start of the last edge to the end point:
      edge_id = result.complete_path[end_edge_index];
      const Edge &e1 = network_.get_edge(edge_id);
      start_offset = 0;
      end_offset = mc1.c.offset;
      line = ALGORITHM::cutoffseg_unique(e1.geom, start_offset, end_offset);
      distances = ALGORITHM::calculate_linestring_euclidean_distances(line);
      start_distance = cumulative_distance;
      points.clear();
      for (int j = 0; j < line.get_num_points(); ++j)
      {
        if (j > 0)
        {
          cumulative_distance += distances[j - 1];
        }
        points.push_back({line.get_x(j), line.get_y(j), cumulative_distance - start_distance, cumulative_distance});
      }
      segment.edges.push_back({edge_id, points});
    }

    // Done!
    output.segments.push_back(segment);
  }
  output.error_code = MatchErrorCode::SUCCESS;
  return output;
}

double FastMapMatch::get_shortest_path_distance(const Candidate *ca, const Candidate *cb, double reverse_tolerance)
{
  double shortest_path_distance = 0;
  if (ca->edge->id == cb->edge->id && ca->offset <= cb->offset)
  {
    // Transition on the same edge, where b is after a i.e. not reversing.
    shortest_path_distance = cb->offset - ca->offset;
  }
  else if (ca->edge->id == cb->edge->id && ca->offset - cb->offset < ca->edge->length * reverse_tolerance)
  {
    // Transition on the same edge, where b is before a but also within the reverse tolerance, then allow.
    shortest_path_distance = 0;
  }
  else if (ca->edge->target == cb->edge->source)
  {
    // Transition on the same OD nodes
    shortest_path_distance = ca->edge->length - ca->offset + cb->offset;
  }
  else
  {
    Record *r = ubodt_->look_up(ca->edge->target, cb->edge->source);
    // No sp path exist from O to D.
    // TODO: could use djikstra here to lookup the sp distance
    if (r == nullptr)
      return std::numeric_limits<double>::infinity();
    // calculate original SP distance
    shortest_path_distance = r->cost + ca->edge->length - ca->offset + cb->offset;
  }
  return shortest_path_distance;
}

int FastMapMatch::update_tg(TransitionGraph *tg, const Trajectory &trajectory, bool *all_connected, double reverse_tolerance)
{
  SPDLOG_DEBUG("Update transition graph");
  std::vector<TGLayer> &layers = tg->get_layers();
  std::vector<double> euclidean_distances = ALGORITHM::calculate_linestring_euclidean_distances(trajectory.geom);
  int N = layers.size();

  for (int i = 0; i < N - 1; ++i)
  {
    SPDLOG_DEBUG("Update layer {} ", i);
    bool layer_connected = false;
    update_layer(i, &(layers[i]), &(layers[i + 1]), euclidean_distances[i], reverse_tolerance, &layer_connected);
    if (!layer_connected)
    {
      SPDLOG_DEBUG("Traj {} unmatched as point {} and {} not connected", trajectory.id, i, i + 1);
      if (all_connected != nullptr)
      {
        *all_connected = false;
      }
      return i;
      // tg->print_optimal_info();
    }
  }
  SPDLOG_DEBUG("Update transition graph done");
  if (all_connected != nullptr)
  {
    *all_connected = true;
  }
  return N - 1;
}

void FastMapMatch::update_layer(int level, TGLayer *la_ptr, TGLayer *lb_ptr, double euclidean_distance, double reverse_tolerance, bool *connected)
{
  // SPDLOG_TRACE("Update layer");
  TGLayer &lb = *lb_ptr;
  bool layer_connected = false;
  for (auto iter_a = la_ptr->begin(); iter_a != la_ptr->end(); ++iter_a)
  {
    NodeIndex source = iter_a->c->index;
    for (auto iter_b = lb_ptr->begin(); iter_b != lb_ptr->end(); ++iter_b)
    {
      double shortest_path_distance = get_shortest_path_distance(iter_a->c, iter_b->c, reverse_tolerance);
      double tp = TransitionGraph::calculate_transition_probability(shortest_path_distance, euclidean_distance);
      double temp = iter_a->cumu_prob + log(tp) + log(iter_b->ep);
      SPDLOG_TRACE("L {} f {} t {} sp {} dist {} tp {} ep {} fcp {} tcp {}",
                   level, iter_a->c->edge->id, iter_b->c->edge->id,
                   shortest_path_distance, euclidean_distance, tp, iter_b->ep, iter_a->cumu_prob,
                   temp);
      if (temp >= iter_b->cumu_prob)
      {
        if (temp > -std::numeric_limits<double>::infinity())
        {
          layer_connected = true;
        }
        iter_b->cumu_prob = temp;
        iter_b->prev = &(*iter_a);
        iter_b->tp = tp;
        iter_b->shortest_path_distance = shortest_path_distance;
      }
    }
  }
  if (connected != nullptr)
  {
    *connected = layer_connected;
  }
  // SPDLOG_TRACE("Update layer done");
}
