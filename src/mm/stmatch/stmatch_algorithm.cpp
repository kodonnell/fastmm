#include "mm/stmatch/stmatch_algorithm.hpp"
#include "algorithm/geom_algorithm.hpp"
#include "util/debug.hpp"
#include "util/util.hpp"

#include <limits>

using namespace FMM;
using namespace FMM::CORE;
using namespace FMM::NETWORK;
using namespace FMM::MM;

STMATCHConfig::STMATCHConfig(
    int k_arg, double candidate_search_radius_arg, double gps_error_arg,
    double vmax_arg, double factor_arg, double reverse_tolerance_arg) : k(k_arg), candidate_search_radius(candidate_search_radius_arg), gps_error(gps_error_arg),
                                                                        vmax(vmax_arg), factor(factor_arg),
                                                                        reverse_tolerance(reverse_tolerance_arg) {
                                                                        };

// Procedure of HMM based map matching algorithm.
MatchResult STMATCH::match_trajectory(const Trajectory &trajectory, const STMATCHConfig &config)
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
    SPDLOG_WARN("No candidates found for trajectory {} at points {}", trajectory.id, unmatched_indices);
    result.error_code = MatchErrorCode::CANDIDATES_NOT_FOUND;
    result.unmatched_candidate_indices = unmatched_indices;
    return result;
  }

  SPDLOG_DEBUG("Generate dummy graph");
  DummyGraph dg(tc, config.reverse_tolerance);
  SPDLOG_DEBUG("Generate composite_graph");
  CompositeGraph cg(graph_, dg);
  SPDLOG_DEBUG("Generate composite_graph");
  TransitionGraph tg(tc, config.gps_error);
  SPDLOG_DEBUG("Update cost in transition graph");
  // The network will be used internally to update transition graph
  update_tg(&tg, cg, trajectory, config);
  SPDLOG_DEBUG("Optimal path inference");
  TGOpath tg_opath = tg.backtrack();
  SPDLOG_DEBUG("Optimal path size {}", tg_opath.size());
  MatchedCandidatePath matched_candidate_path(tg_opath.size());
  std::transform(tg_opath.begin(), tg_opath.end(),
                 matched_candidate_path.begin(),
                 [](const TGNode *a)
                 {
                   return MatchedCandidate{
                       *(a->c), a->ep, a->tp, a->shortest_path_distance};
                 });
  OptimalPath optimal_path(tg_opath.size());
  std::transform(tg_opath.begin(), tg_opath.end(),
                 optimal_path.begin(),
                 [](const TGNode *a)
                 {
                   return a->c->edge->id;
                 });
  std::vector<int> indices;
  CompletePath complete_path = build_cpath(tg_opath, &indices, config.reverse_tolerance);
  SPDLOG_DEBUG("Opath is {}", optimal_path);
  SPDLOG_DEBUG("Indices is {}", indices);
  SPDLOG_DEBUG("Complete path is {}", complete_path);
  LineString matched_geometry = network_.complete_path_to_geometry(
      trajectory.geom, complete_path);
  return MatchResult{
      trajectory.id, matched_candidate_path, optimal_path, complete_path, indices, matched_geometry};
}

void STMATCH::update_tg(TransitionGraph *tg,
                        const CompositeGraph &cg,
                        const Trajectory &trajectory,
                        const STMATCHConfig &config)
{
  SPDLOG_DEBUG("Update transition graph");
  std::vector<TGLayer> &layers = tg->get_layers();
  std::vector<double> euclidean_distances = ALGORITHM::calculate_linestring_euclidean_distances(trajectory.geom);
  int N = layers.size();
  for (int i = 0; i < N - 1; ++i)
  {
    // Routing from current_layer to next_layer
    double delta = 0;
    if (trajectory.timestamps.size() != N)
    {
      delta = euclidean_distances[i] * config.factor * 4;
    }
    else
    {
      double duration = trajectory.timestamps[i + 1] - trajectory.timestamps[i];
      delta = config.factor * config.vmax * duration;
    }
    update_layer(i, &(layers[i]), &(layers[i + 1]),
                 cg, euclidean_distances[i], delta);
  }
  SPDLOG_DEBUG("Update transition graph done");
}

void STMATCH::update_layer(int level, TGLayer *la_ptr, TGLayer *lb_ptr,
                           const CompositeGraph &cg,
                           double euclidean_distance,
                           double delta)
{
  SPDLOG_DEBUG("Update layer {} starts", level);
  TGLayer &lb = *lb_ptr;
  for (auto iter_a = la_ptr->begin(); iter_a != la_ptr->end(); ++iter_a)
  {
    NodeIndex source = iter_a->c->index;
    // SPDLOG_TRACE("  Calculate distance from source {}", source);
    // single source upper bound routing
    std::vector<NodeIndex> targets(lb.size());
    std::transform(lb.begin(), lb.end(), targets.begin(),
                   [](TGNode &a)
                   {
                     return a.c->index;
                   });
    std::vector<double> distances = shortest_path_upperbound(
        level, cg, source, targets, delta);
    for (auto iter_b = lb_ptr->begin(); iter_b != lb_ptr->end(); ++iter_b)
    {
      int i = std::distance(lb_ptr->begin(), iter_b);
      double tp = TransitionGraph::calculate_transition_probability(distances[i], euclidean_distance);
      double temp = iter_a->cumu_prob + log(tp) + log(iter_b->ep);
      SPDLOG_TRACE("L {} f {} t {} sp {} dist {} tp {} ep {} fcp {} tcp {}",
                   level, iter_a->c->edge->id, iter_b->c->edge->id,
                   distances[i], euclidean_distance, tp, iter_b->ep, iter_a->cumu_prob,
                   temp);
      if (temp >= iter_b->cumu_prob)
      {
        iter_b->cumu_prob = temp;
        iter_b->prev = &(*iter_a);
        iter_b->shortest_path_distance = distances[i];
        iter_b->tp = tp;
      }
    }
  }
  SPDLOG_DEBUG("Update layer done");
}

std::vector<double> STMATCH::shortest_path_upperbound(
    int level, const CompositeGraph &cg, NodeIndex source,
    const std::vector<NodeIndex> &targets, double delta)
{
  // SPDLOG_TRACE("Upperbound shortest path source {}", source);
  // SPDLOG_TRACE("Upperbound shortest path targets {}", targets);
  std::unordered_set<NodeIndex> unreached_targets;
  for (auto &node : targets)
  {
    unreached_targets.insert(node);
  }
  DistanceMap dmap;
  PredecessorMap pmap;
  Heap Q;
  Q.push(source, 0);
  pmap.insert({source, source});
  dmap.insert({source, 0});
  double temp_dist = 0;
  // Dijkstra search
  while (!Q.empty() && !unreached_targets.empty())
  {
    HeapNode node = Q.top();
    Q.pop();
    // SPDLOG_TRACE("  Node u {} dist {}", node.index, node.value);
    NodeIndex u = node.index;
    auto iter = unreached_targets.find(u);
    if (iter != unreached_targets.end())
    {
      // Remove u
      // SPDLOG_TRACE("  Remove target {}", u);
      unreached_targets.erase(iter);
    }
    if (node.value > delta)
      break;
    std::vector<CompEdgeProperty> out_edges = cg.out_edges(u);
    for (auto node_iter = out_edges.begin(); node_iter != out_edges.end();
         ++node_iter)
    {
      NodeIndex v = node_iter->v;
      temp_dist = node.value + node_iter->cost;
      // SPDLOG_TRACE("  Examine node v {} temp dist {}", v, temp_dist);
      auto v_iter = dmap.find(v);
      if (v_iter != dmap.end())
      {
        // dmap contains node v
        if (v_iter->second - temp_dist > 1e-6)
        {
          // a smaller distance is found for v
          // SPDLOG_TRACE("    Update key {} {} in pdmap prev dist {}",
          //              v, temp_dist, v_iter->second);
          pmap[v] = u;
          dmap[v] = temp_dist;
          Q.decrease_key(v, temp_dist);
        }
      }
      else
      {
        // dmap does not contain v
        if (temp_dist <= delta)
        {
          // SPDLOG_TRACE("    Insert key {} {} into pmap and dmap",
          //              v, temp_dist);
          Q.push(v, temp_dist);
          pmap.insert({v, u});
          dmap.insert({v, temp_dist});
        }
      }
    }
  }
  // Update distances
  // SPDLOG_TRACE("  Update distances");
  std::vector<double> distances;
  for (int i = 0; i < targets.size(); ++i)
  {
    if (dmap.find(targets[i]) != dmap.end())
    {
      distances.push_back(dmap[targets[i]]);
    }
    else
    {
      distances.push_back(std::numeric_limits<double>::max());
    }
  }
  // SPDLOG_TRACE("  Distance value {}", distances);
  return distances;
}

CompletePath STMATCH::build_cpath(const TGOpath &optimal_path, std::vector<int> *indices,
                                  double reverse_tolerance)
{
  SPDLOG_DEBUG("Build complete_path from optimal candidate path");
  CompletePath complete_path;
  if (!indices->empty())
    indices->clear();
  if (optimal_path.empty())
    return complete_path;
  const std::vector<Edge> &edges = network_.get_edges();
  int N = optimal_path.size();
  complete_path.push_back(optimal_path[0]->c->edge->id);
  int current_idx = 0;
  // SPDLOG_TRACE("Insert index {}", current_idx);
  indices->push_back(current_idx);
  for (int i = 0; i < N - 1; ++i)
  {
    const Candidate *a = optimal_path[i]->c;
    const Candidate *b = optimal_path[i + 1]->c;
    // SPDLOG_TRACE("Check a {} b {}", a->edge->id, b->edge->id);
    if ((a->edge->id != b->edge->id) ||
        (a->offset - b->offset > a->edge->length * reverse_tolerance))
    {
      auto segs = graph_.shortest_path_dijkstra(a->edge->target,
                                                b->edge->source);
      // No transition found
      if (segs.empty() && a->edge->target != b->edge->source)
      {
        SPDLOG_TRACE("Candidate {} has disconnected edge {} to {}",
                     i, a->edge->id, b->edge->id);
        indices->clear();
        return {};
      }
      for (int e : segs)
      {
        complete_path.push_back(edges[e].id);
        ++current_idx;
      }
      complete_path.push_back(b->edge->id);
      ++current_idx;
      indices->push_back(current_idx);
    }
    else
    {
      indices->push_back(current_idx);
    }
  }
  SPDLOG_DEBUG("Build complete_path from optimal candidate path done");
  return complete_path;
}
