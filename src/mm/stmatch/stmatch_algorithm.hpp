/**
 * Fast map matching.
 *
 * Stmatch algorithm implementation and configuration
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */
#ifndef FMM_STMATCH_ALGORITHM_HPP
#define FMM_STMATCH_ALGORITHM_HPP

#include "network/network.hpp"
#include "network/network_graph.hpp"
#include "mm/composite_graph.hpp"
#include "mm/transition_graph.hpp"
#include "mm/mm_type.hpp"

#include <string>
#include <boost/property_tree/ptree.hpp>

#include "cxxopts/cxxopts.hpp"

namespace FMM
{
  namespace MM
  {

    /**
     * Configuration of stmatch algorithm
     */
    struct STMATCHConfig
    {
      /**
       * Constructor of stmatch algorithm configuration
       * @param k_arg the number of candidates
       * @param candidate_search_radius_arg the search radius, in map unit, which is the same as
       * GPS data and network data.
       * @param gps_error_arg the gps error, in map unit
       * @param vmax_arg the maximum speed of the vehicle in map unit/second
       * @param factor_arg a factor multiplied with vmax*deltaT to constrain the
       * search in stmatch.
       */
      STMATCHConfig(int k_arg = 8, double candidate_search_radius_arg = 50, double gps_error_arg = 50,
                    double vmax_arg = 30, double factor_arg = 1.5,
                    double reverse_tolerance_arg = 0.0);
      int k;                          /**< number of candidates */
      double candidate_search_radius; /**< search radius for candidates, unit is map_unit*/
      double gps_error;               /**< GPS error, unit is map_unit */
      double vmax;                    /**< maximum speed of the vehicle, unit is map_unit/second */
      double factor;                  /**< factor multiplied to vmax*deltaT to
                                           limit the search of shortest path */
      double reverse_tolerance;
    };

    /**
     * %STMATCH algorithm/model
     */
    class STMATCH
    {
    public:
      /**
       * Create a stmatch model from network and graph
       */
      STMATCH(const NETWORK::Network &network, const NETWORK::NetworkGraph &graph) : network_(network), graph_(graph) {
                                                                                     };
      /**
       * Match a trajectory to the road network
       * @param  trajectory   input trajector data
       * @param  config configuration of stmatch algorithm
       * @return map matching result
       */
      MatchResult match_trajectory(const CORE::Trajectory &trajectory,
                                   const STMATCHConfig &config);

    protected:
      /**
       * Update probabilities in a transition graph
       * @param tg transition graph
       * @param cg composition graph
       * @param trajectory raw trajectory
       * @param config map match configuration
       */
      void update_tg(TransitionGraph *tg,
                     const CompositeGraph &cg,
                     const CORE::Trajectory &trajectory,
                     const STMATCHConfig &config);
      /**
       * Update probabilities between two layers a and b in the transition graph
       * @param level   the index of layer a
       * @param la_ptr  layer a
       * @param lb_ptr  layer b next to a
       * @param cg      Composition graph
       * @param euclidean_distance Euclidean distance between two observed point
       * @param delta   An upper bound to limit the search
       */
      void update_layer(int level, TGLayer *la_ptr, TGLayer *lb_ptr,
                        const CompositeGraph &cg,
                        double euclidean_distance,
                        double delta);

      /**
       * Return distances from source to all targets and with an upper bound of
       * delta to stop the search
       * @param  level   The source node's level in transiton graph, used for
       * logging.
       * @param  cg      Composition graph
       * @param  source  Source node
       * @param  targets A vector of target nodes
       * @param  delta   An upper bound value to constrain the search
       * @return A vector of distances to the target nodes, if any target node
       * is not reached, infinity distance will be returned for that node.
       */
      std::vector<double> shortest_path_upperbound(
          int level,
          const CompositeGraph &cg, NETWORK::NodeIndex source,
          const std::vector<NETWORK::NodeIndex> &targets, double delta);

      /**
       * Create a topologically connected path according to each matched
       * candidate
       * @param  tg_opath A sequence of optimal candidate nodes
       * @param  indices  the indices to be updated to store the index of matched
       * edge or candidate in the returned path.
       * @return A vector of edge id representing the traversed path
       */
      CompletePath build_cpath(const TGOpath &tg_opath, std::vector<int> *indices,
                               double reverse_tolerance = 0);

    private:
      const NETWORK::Network &network_;
      const NETWORK::NetworkGraph &graph_;
    }; // STMATCH
  }
} // FMM

#endif
