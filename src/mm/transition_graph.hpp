/**
 * Fast map matching.
 *
 * Definition of a TransitionGraph, which is a wrapper of trajectory
 * candidates, raw trajectory and UBODT.
 * This class is designed for optimal path inference where
 * Viterbi algorithm is implemented.
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FASTMM_TRANSITION_GRAPH_HPP
#define FASTMM_TRANSITION_GRAPH_HPP

#include "network/type.hpp"
#include "mm/mm_type.hpp"

#include <float.h>

namespace FASTMM
{

  namespace MM
  {

    /**
     * A node in the transition graph
     */
    struct TGNode
    {
      const Candidate *c;            /**< Candidate */
      TGNode *prev;                  /**< previous optimal candidate */
      double ep;                     /**< emission probability */
      double tp;                     /**< transition probability from previous optimal candidate */
      double cumu_prob;              /**< current node's accumulative probability */
      double shortest_path_distance; /**< shorest path distance from previous optimal
                           candidate to current node */
    };

    /**
     * A layer of nodes in the transition graph.
     */
    typedef std::vector<TGNode> TGLayer;
    /**
     * The optimal path of nodes in the transition graph
     */
    typedef std::vector<const TGNode *> TGOpath;

    /**
     * Transition graph class in HMM.
     *
     * The class stores the underlying transition graph of a HMM, which stores
     * the probabilities of candidates matched to a trajectories.
     */
    class TransitionGraph
    {
    public:
      /**
       * Transition graph constructor.
       *
       * A transition graph is created to store the probability of
       * emission and transition in HMM model where optimal path will be
       * infered.
       *
       * @param tc        Trajectory candidates
       * @param gps_error GPS error
       */
      TransitionGraph(const TrajectoryCandidates &tc, double gps_error);

      /**
       * Calculate transition probability for SHORTEST mode (distance-based)
       * @param  path_distance Shortest path distance between two candidates
       * @param  euclidean_distance Euclidean distance between two candidates
       * @return transition probability in HMM
       */
      static double get_shortest_transition_probability(double path_distance, double euclidean_distance);

      /**
       * Calculate transition probability for FASTEST mode (time-based)
       * @param  path_time Path travel time between two candidates
       * @param  euclidean_distance Euclidean distance between two candidates
       * @param  reference_speed Reference speed for calculating expected time
       * @return transition probability in HMM
       */
      static double get_fastest_transition_probability(double path_time, double euclidean_distance, double reference_speed);

      /**
       * Calculate emission probability
       * @param  dist  The actual gps error from observed point to matched point
       * @param  error The GPS sensor's accuracy
       * @return  emission probability in HMM
       */
      static double calc_ep(double dist, double error);

      /**
       * Reset all the proability data stored in a layer of the transition graph
       * @param layer A layer in the transition graph
       */
      void reset_layer(TGLayer *layer);

      /**
       * Find the optimal candidate in a layer with
       * the highest accumulative probability.
       * @param  layer [description]
       * @return  pointer to the optimal node in the transition graph
       */
      const TGNode *find_optimal_candidate(const TGLayer &layer);
      /**
       * Backtrack the transition graph to find an optimal path
       * @return An optimal path connecting the first layer with last layer and
       * has the highest accumulative probability value.
       */
      TGOpath backtrack();
      /**
       * Get a reference to the inner layers of the transition graph.
       */
      std::vector<TGLayer> &get_layers();
      void print_optimal_info();

    private:
      // candidates of a trajectory
      std::vector<TGLayer> layers;
    };

  }
}
#endif /* FASTMM_TRANSITION_GRAPH_HPP */
