/**
 * Fast map matching.
 *
 * Definition of map matching types
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FASTMM_INCLUDE_FASTMM_FASTMM_RESULT_HPP_
#define FASTMM_INCLUDE_FASTMM_FASTMM_RESULT_HPP_

#include "network/type.hpp"

namespace FASTMM
{

  /**
   * Class related with map matching
   */
  namespace MM
  {

    /**
     * Error codes for map matching results
     */
    enum class MatchErrorCode : int
    {
      SUCCESS = 0,                 /**< Matching succeeded */
      CANDIDATES_NOT_FOUND = 1,    /**< No candidate edges found for trajectory */
      DISCONNECTED_LAYERS = 2,     /**< Trajectory has disconnected layers (can't find valid path between some GPS points) */
      INDEX_OUT_OF_BOUNDS = 3,     /**< Internal error: start edge index out of bounds */
      INDEX_OUT_OF_BOUNDS_END = 4, /**< Internal error: end edge index out of bounds */
      UNKNOWN_ERROR = 255          /**< Unknown error occurred */
    };

    /**
     * %Candidate edge matched to a GPS point
     */
    struct Candidate
    {
      FASTMM::NETWORK::NodeIndex index; /**< The index is defined for a specific candidate the index starting from N where N is the number of vertices in the graph */
      double offset;                    /**< offset distance from the start of polyline to p' */
      double dist;                      /**< distance from original point p to map matched point p' */
      NETWORK::Edge *edge;              /**< candidate edge */
      FASTMM::CORE::Point point;        /**< boost point */
    };

    typedef std::vector<Candidate> PointCandidates; /**< Point candidates */
    typedef std::vector<PointCandidates> TrajectoryCandidates;
    /**< trajectory  candidates */

    typedef std::vector<const Candidate *> OptCandidatePath;
    /**< Optimal candidates*/

    typedef std::vector<FASTMM::NETWORK::EdgeID> OptimalPath;  /**< Optimal path, edge id matched to each point in the trajectory */
    typedef std::vector<FASTMM::NETWORK::EdgeID> CompletePath; /**< Complete path, ids of a sequence of topologically connected edges.*/

    /**
     * A candidate matched to a point
     */
    struct MatchedCandidate
    {
      Candidate c;                   /**< Candidate matched to a point */
      double ep;                     /**< emission probability */
      double tp;                     /**< transition probability to previous matched candidate */
      double shortest_path_distance; /**< shortest path distance to previous matched candidate */
    };

    /**
     * A vector of candidates, used for representing the matching result of
     * a trajectory.
     */
    typedef std::vector<MatchedCandidate> MatchedCandidatePath;

    /**
     * Map matched result representation
     */
    struct MatchResult
    {
      int id;                                       /**< id of the trajectory to be matched */
      MatchedCandidatePath opt_candidate_path;      /**< A vector of candidate matched to each point of a trajectory. It is stored in order to export more detailed map matching information. */
      OptimalPath optimal_path;                     /**< the optimal path, containing id of edges matched to each point in a trajectory */
      CompletePath complete_path;                   /**< the complete path, containing ids of a sequence of topologically connected edges traversed by the trajectory.  */
      std::vector<int> indices;                     /**< index of optimal_path edge in complete_path */
      CORE::LineString matched_geometry;            /**< the geometry of the matched path */
      MatchErrorCode error_code;                    /**< error code if the map matching failed */
      int last_connected_trajectory_point;          /**< the index of the last connected layer */
      std::vector<int> unmatched_candidate_indices; /**< the indices of the unmatched trajectory points if no candidates found */
    };

    struct PyMatchPoint
    {
      double x;
      double y;
      double edge_offset;
      double cumulative_distance;
    };

    struct PyMatchSegmentEdge
    {
      long long edge_id;
      std::vector<PyMatchPoint> points;
    };

    struct PyMatchCandidate
    {
      double x;
      double y;
      double t;
      double perpendicular_distance_to_matched_geometry;
      double offset_from_start_of_edge;
    };
    struct PyMatchSegment
    {
      PyMatchCandidate p0;
      PyMatchCandidate p1;
      std::vector<PyMatchSegmentEdge> edges;
    };

    /**
     * Py map matched result representation
     */
    struct PyMatchResult
    {
      int id;                                       /**< id of the trajectory to be matched */
      MatchErrorCode error_code;                    /**< error code if the map matching failed */
      int last_connected_trajectory_point;          /**< the index of the last connected layer */
      std::vector<int> unmatched_candidate_indices; /**< the indices of the unmatched trajectory points if no candidates found */
      std::vector<PyMatchSegment> segments;         /**< A vector of matched segments */
    };

  };

};

#endif // FASTMM_INCLUDE_FASTMM_FASTMM_RESULT_HPP_
