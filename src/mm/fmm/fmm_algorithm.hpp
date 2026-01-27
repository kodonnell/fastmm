/**
 * Fast map matching.
 *
 * fmm algorithm implementation
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FASTMM_FASTMM_ALGORITHM_H_
#define FASTMM_FASTMM_ALGORITHM_H_

#include "network/network.hpp"
#include "network/network_graph.hpp"
#include "mm/transition_graph.hpp"
#include "mm/fmm/ubodt.hpp"

#include <string>
#include <optional>
#include <boost/property_tree/ptree.hpp>

namespace FASTMM
{
  namespace MM
  {
    /**
     * Routing mode for map matching
     */
    enum class TransitionMode
    {
      SHORTEST, /**< Distance-based routing */
      FASTEST   /**< Time-based routing */
    };

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
       * @param reverse_tolerance reverse tolerance
       * @param transition_mode routing mode (SHORTEST or FASTEST)
       * @param reference_speed reference speed for FASTEST mode (required if mode is FASTEST)
       */
      FastMapMatchConfig(int k_arg = 8,
                         double candidate_search_radius = 50,
                         double gps_error = 50,
                         double reverse_tolerance = 0.0,
                         TransitionMode transition_mode = TransitionMode::SHORTEST,
                         std::optional<double> reference_speed = std::nullopt);
      int k;                                 /**< Number of candidates */
      double candidate_search_radius;        /**< Search radius*/
      double gps_error;                      /**< GPS error */
      double reverse_tolerance;              /**< Reverse tolerance */
      TransitionMode transition_mode;        /**< Routing mode */
      std::optional<double> reference_speed; /**< Reference speed for FASTEST mode */
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
      /**
       * Match a trajectory with automatic splitting on failures
       * Performs candidate search once and reuses it for all sub-trajectories
       * @param  trajectory   input trajectory data
       * @param  config configuration of map matching algorithm
       * @return split match result containing all successful and failed sub-trajectories
       */
      PySplitMatchResult pymatch_trajectory_split(const CORE::Trajectory &trajectory,
                                                  const FastMapMatchConfig &config);

    protected:
      /**
       * Get path distance between two candidates (always distance, regardless of mode)
       * @param  ca from candidate
       * @param  cb to candidate
       * @param  reverse_tolerance reverse tolerance parameter
       * @return  path distance
       */
      double get_distance(const Candidate *ca,
                          const Candidate *cb,
                          double reverse_tolerance);

      /**
       * Get path time between two candidates (distance/speed for each segment)
       * @param  ca from candidate
       * @param  cb to candidate
       * @param  reverse_tolerance reverse tolerance parameter
       * @return  path time
       */
      double get_time(const Candidate *ca,
                      const Candidate *cb,
                      double reverse_tolerance);

      /**
       * Update probabilities in a transition graph
       * @param tg transition graph
       * @param trajectory raw trajectory
       * @param config map match configuration
       * @param all_connected output parameter indicating if all layers are connected
       * @return idx of the last connected layer
       */
      int update_tg(TransitionGraph *tg, const CORE::Trajectory &trajectory, const FastMapMatchConfig &config, bool *all_connected);
      /**
       * Update probabilities between two layers a and b in the transition graph
       * @param level   the index of layer a
       * @param la_ptr  layer a
       * @param lb_ptr  layer b next to a
       * @param euclidean_distance Euclidean distance between two observed point
       * @param config map match configuration
       * @param connected the variable is set to false if the layer is not connected
       * with the next layer
       */
      void update_layer(int level, TGLayer *la_ptr, TGLayer *lb_ptr,
                        double euclidean_distance,
                        const FastMapMatchConfig &config,
                        bool *connected);
      /**
       * Build PyMatchSegment objects from matched path
       * Helper function for building segment output, extracted to avoid duplication
       * @param matched_path matched candidate path
       * @param complete_path complete edge path
       * @param indices indices mapping
       * @param trajectory original trajectory
       * @param start_idx starting trajectory index
       * @param end_idx ending trajectory index
       * @return vector of PyMatchSegment objects
       */
      std::vector<PyMatchSegment> build_py_segments(const MatchedCandidatePath &matched_path,
                                                    const CompletePath &complete_path,
                                                    const std::vector<int> &indices,
                                                    const CORE::Trajectory &trajectory,
                                                    int start_idx,
                                                    int end_idx);

    private:
      const NETWORK::Network &network_;
      const NETWORK::NetworkGraph &graph_;
      std::shared_ptr<UBODT> ubodt_;
    };
  }
}

#endif // FASTMM_FASTMM_ALGORITHM_H_
