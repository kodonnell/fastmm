/**
 * Fast map matching.
 *
 * Definition of a TransitionGraph.
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#include "mm/transition_graph.hpp"
#include "network/type.hpp"
#include "util/debug.hpp"

using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::NETWORK;
using namespace FASTMM::MM;

TransitionGraph::TransitionGraph(const TrajectoryCandidates &tc, double gps_error)
{
  for (auto cs = tc.begin(); cs != tc.end(); ++cs)
  {
    TGLayer layer;
    for (auto iter = cs->begin(); iter != cs->end(); ++iter)
    {
      double ep = calc_ep(iter->dist, gps_error);
      layer.push_back(TGNode{&(*iter), nullptr, ep, 0, -std::numeric_limits<double>::infinity(), 0});
    }
    layers.push_back(layer);
  }
  if (!tc.empty())
  {
    reset_layer(&(layers[0]));
  }
}

double TransitionGraph::calculate_transition_probability(double shortest_path_distance, double euclidean_distance)
{
  return euclidean_distance >= shortest_path_distance ? 1.0 : euclidean_distance / shortest_path_distance;
}

double TransitionGraph::calc_ep(double dist, double error)
{
  double a = dist / error;
  return exp(-0.5 * a * a);
}

// Reset the properties of a candidate set
void TransitionGraph::reset_layer(TGLayer *layer)
{
  for (auto iter = layer->begin(); iter != layer->end(); ++iter)
  {
    iter->cumu_prob = log(iter->ep);
    iter->prev = nullptr;
  }
}

const TGNode *TransitionGraph::find_optimal_candidate(const TGLayer &layer)
{
  const TGNode *opt_c = nullptr;
  double final_prob = -std::numeric_limits<double>::infinity();
  for (auto c = layer.begin(); c != layer.end(); ++c)
  {
    if (final_prob < c->cumu_prob)
    {
      final_prob = c->cumu_prob;
      opt_c = &(*c);
    }
  }
  return opt_c;
}

TGOpath TransitionGraph::backtrack()
{
  SPDLOG_TRACE("Backtrack on transition graph");
  TGNode *track_cand = nullptr;
  double final_prob = -std::numeric_limits<double>::infinity();
  std::vector<TGNode> &last_layer = layers.back();
  for (auto c = last_layer.begin(); c != last_layer.end(); ++c)
  {
    if (final_prob < c->cumu_prob)
    {
      final_prob = c->cumu_prob;
      track_cand = &(*c);
    }
  }
  TGOpath optimal_path;
  int i = layers.size();
  if (final_prob > -std::numeric_limits<double>::infinity())
  {
    optimal_path.push_back(track_cand);
    --i;
    SPDLOG_TRACE("Optimal candidate {} edge id {} sp {} tp {} cp {}",
                 i, track_cand->c->edge->id, track_cand->shortest_path_distance, track_cand->tp,
                 track_cand->cumu_prob);
    // Iterate from tail to head to assign path
    while ((track_cand = track_cand->prev) != nullptr)
    {
      optimal_path.push_back(track_cand);
      --i;
      SPDLOG_TRACE("Optimal candidate {} edge id {} sp {} tp {} cp {}",
                   i, track_cand->c->edge->id, track_cand->shortest_path_distance, track_cand->tp,
                   track_cand->cumu_prob);
    }
    std::reverse(optimal_path.begin(), optimal_path.end());
  }
  SPDLOG_TRACE("Backtrack on transition graph done");
  return optimal_path;
}

void TransitionGraph::print_optimal_info()
{
  int N = layers.size();
  if (N < 1)
    return;
  const TGNode *global_opt_node = nullptr;
  for (int i = N - 1; i >= 0; --i)
  {
    const TGNode *local_opt_node = find_optimal_candidate(layers[i]);
    if (global_opt_node != nullptr)
    {
      global_opt_node = global_opt_node->prev;
    }
    else
    {
      global_opt_node = local_opt_node;
    }
    SPDLOG_TRACE("Point {} global opt {} local opt {}",
                 i, (global_opt_node == nullptr) ? -1 : global_opt_node->c->edge->id,
                 (local_opt_node == nullptr) ? -1 : local_opt_node->c->edge->id);
  }
};

std::vector<TGLayer> &TransitionGraph::get_layers()
{
  return layers;
}
