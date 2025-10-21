/**
 * Fast map matching.
 *
 * fmm algorithm implementation
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FMM_FMM_ALGORITHM_H_
#define FMM_FMM_ALGORITHM_H_

#include "network/network.hpp"
#include "network/network_graph.hpp"
#include "mm/transition_graph.hpp"
#include "mm/fmm/ubodt.hpp"

#include <string>
#include <boost/property_tree/ptree.hpp>

namespace FMM
{
  namespace MM
  {
    /**
     * Configuration class for fmm algorithm
     */
    struct FastMapMatchConfig
    {
      /**
       * Constructor of FastMapMatch configuration
       * @param k_arg the number of candidates
       * @param candidate_search_radius the search radius, in map unit, which is the same as
       * GPS data and network data.
       * @param gps_error the gps error, in map unit
       *
       */
      FastMapMatchConfig(int k_arg = 8, double candidate_search_radius = 50, double gps_error = 50, double reverse_tolerance = 0.0);
      int k;                          /**< Number of candidates */
      double candidate_search_radius; /**< Search radius*/
      double gps_error;               /**< GPS error */
      double reverse_tolerance;
    };

    /**
     * Fast map matching algorithm/model.
     */
    class FastMapMatch
    {
    public:
      /**
       * Constructor of Fast map matching model
       * @param network road network
       * @param graph road network graph
       * @param ubodt Upperbounded origin destination table
       */
      FastMapMatch(const NETWORK::Network &network,
                   const NETWORK::NetworkGraph &graph,
                   std::shared_ptr<UBODT> ubodt)
          : network_(network), graph_(graph), ubodt_(ubodt) {
            };
      /**
       * Match a trajectory to the road network
       * @param  trajectory   input trajector data
       * @param  config configuration of map matching algorithm
       * @return map matching result
       */
      MatchResult match_trajectory(const CORE::Trajectory &trajectory,
                                   const FastMapMatchConfig &config);
      /**
       * Match a trajectory to the road network
       * @param  trajectory   input trajector data
       * @param  config configuration of map matching algorithm
       * @return map matching result
       */
      PyMatchResult pymatch_trajectory(const CORE::Trajectory &trajectory,
                                       const FastMapMatchConfig &config);

    protected:
      /**
       * Get shortest path distance between two candidates
       * @param  ca from candidate
       * @param  cb to candidate
       * @return  shortest path value
       */
      double get_shortest_path_distance(const Candidate *ca,
                                        const Candidate *cb,
                                        double reverse_tolerance);
      /**
       * Update probabilities in a transition graph
       * @param tg transition graph
       * @param trajectory raw trajectory
       * @param config map match configuration
       * @return idx of the last connected layer
       */
      int update_tg(TransitionGraph *tg, const CORE::Trajectory &trajectory, bool *all_connected, double reverse_tolerance = 0);
      /**
       * Update probabilities between two layers a and b in the transition graph
       * @param level   the index of layer a
       * @param la_ptr  layer a
       * @param lb_ptr  layer b next to a
       * @param euclidean_distance Euclidean distance between two observed point
       * @param connected the variable is set to false if the layer is not connected
       * with the next layer
       */
      void update_layer(int level, TGLayer *la_ptr, TGLayer *lb_ptr,
                        double euclidean_distance, double reverse_tolerance,
                        bool *connected);

    private:
      const NETWORK::Network &network_;
      const NETWORK::NetworkGraph &graph_;
      std::shared_ptr<UBODT> ubodt_;
    };
  }
}

#endif // FMM_FMM_ALGORITHM_H_
