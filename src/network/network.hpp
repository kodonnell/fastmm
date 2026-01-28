/**
 * Fast map matching.
 *
 * Network class
 *
 * @author: Can Yang
 * @version: 2017.11.11
 */

#ifndef FASTMM_NETWORK_HPP
#define FASTMM_NETWORK_HPP

#include "network/type.hpp"
#include "core/gps.hpp"
#include "mm/mm_type.hpp"
#include <iostream>
#include <math.h> // Calulating probability
#include <iomanip>
#include <algorithm>     // Partial sort copy
#include <unordered_set> // Partial sort copy

// Data structures for Rtree
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/iterator/function_output_iterator.hpp>

namespace FASTMM
{
  /**
   * Classes related with network and graph
   */
  namespace NETWORK
  {
    /**
     * Road network class
     */
    class Network
    {
    public:
      /**
       * Box of a edge
       */
      typedef boost::geometry::model::box<FASTMM::CORE::Point> boost_box;
      /**
       * Item stored in a node of Rtree
       */
      typedef std::pair<boost_box, Edge *> Item;
      /**
       * Rtree of road edges
       */
      typedef boost::geometry::index::rtree<
          Item, boost::geometry::index::quadratic<16>>
          Rtree;
      /**
       *  Default constructor of Network (creates empty network)
       *
       *  Creates an empty network that can be populated using add_edge.
       *  After adding all edges, call finalize() to build the spatial index.
       */
      Network();
      /**
       * Get number of nodes in the network
       * @return number of nodes
       */
      int get_node_count() const;
      /**
       * Get number of edges in the network
       * @return number of edges
       */
      int get_edge_count() const;
      bool is_finalized() const;
      const Edge &get_edge(EdgeID id) const;
      const Edge &get_edge(EdgeIndex index) const;
      /**
       * Get edges in the network
       * @return a constant reference to the edges
       */
      const std::vector<Edge> &get_edges() const;
      /**
       * Get edge ID from index
       * @param index index of edge
       * @return edge ID
       */
      EdgeID get_edge_id(EdgeIndex index) const;
      /**
       * Get edge index from ID
       * @param id edge id
       * @return edge index
       */
      EdgeIndex get_edge_index(EdgeID id) const;
      /**
       * Get node ID from index
       * @param index index of node
       * @return node ID
       */
      NodeID get_node_id(NodeIndex index) const;
      /**
       * Get node index from id
       * @param id node id
       * @return node index
       */
      NodeIndex get_node_index(NodeID id) const;
      /**
       * Get node geometry from index
       * @param index node index
       * @return point of a node
       */
      FASTMM::CORE::Point get_node_geom_from_idx(NodeIndex index) const;

      /**
       *  Search for k nearest neighboring (KNN) candidates of a
       *  trajectory within a search radius
       *
       *  @param trajectory: input trajectory
       *  @param k: the number of candidates
       *  @param radius: the search radius
       *  @return a pair of TrajectoryCandidates and vector of unmatched candidate indices
       *
       */
      FASTMM::MM::TrajectoryCandidates search_tr_cs_knn(FASTMM::CORE::Trajectory &trajectory, std::size_t k, double radius) const;

      /**
       * Search for k nearest neighboring (KNN) candidates of a
       * linestring within a search radius
       *
       * @param geom
       * @param k number of candidates
       * @param radius search radius
       * @return a pair of TrajectoryCandidates and vector of unmatched candidate indices
       */
      FASTMM::MM::TrajectoryCandidates search_tr_cs_knn(const FASTMM::CORE::LineString &geom, std::size_t k, double radius) const;
      /**
       * Get edge geometry
       * @param edge_id edge id
       * @return Geometry of edge
       */
      const FASTMM::CORE::LineString &get_edge_geom(EdgeID edge_id) const;
      /**
       * Extract the geometry of a complete path, whose two end segment will be
       * clipped according to the input trajectory
       * @param traj input trajectory
       * @param complete_path complete path
       */
      FASTMM::CORE::LineString complete_path_to_geometry(
          const FASTMM::CORE::LineString &traj,
          const MM::CompletePath &complete_path) const;
      /**
       * Get all node geometry
       * @return a vector of points
       */
      const std::vector<FASTMM::CORE::Point> &get_vertex_points() const;
      /**
       * Get node geometry
       * @param u node index
       * @return geometry of node
       */
      const FASTMM::CORE::Point &get_vertex_point(NodeIndex u) const;
      /**
       * Extract the geometry of a route in the network
       * @param path a route stored with edge ID
       * @return the geometry of the route
       */
      FASTMM::CORE::LineString route2geometry(const std::vector<EdgeID> &path) const;
      /**
       * Extract the geometry of a route in the network
       * @param path a route stored with edge Index
       * @return the geometry of the route
       */
      FASTMM::CORE::LineString route2geometry(
          const std::vector<EdgeIndex> &path) const;
      /**
       * Compare two candidate according to their GPS error
       * @param a candidate a
       * @param b candidate b
       * @return true if a.dist<b.dist
       */
      static bool candidate_compare(const MM::Candidate &a, const MM::Candidate &b);
      /**
       * Add an edge to the network
       * @param edge_id the ID of the edge
       * @param source the source node ID
       * @param target the target node ID
       * @param geom the geometry of the edge as a LineString
       * @param speed optional speed limit on the edge (for FASTEST mode)
       */
      void add_edge(EdgeID edge_id, NodeID source, NodeID target,
                    const FASTMM::CORE::LineString &geom,
                    std::optional<double> speed = std::nullopt);
      /**
       * Build rtree spatial index for the network
       *
       * This must be called after all edges have been added to the network
       * and before performing any spatial queries.
       */
      void finalize();

      /**
       * Compute a hash of the network structure for cache validation
       *
       * @return 32-character hex hash string
       */
      std::string compute_hash() const;

    private:
      /**
       * Concatenate a linestring segs to a linestring line, used in the
       * function complete_path_to_geometry
       *
       * @param line: linestring which will be updated
       * @param segs: segs that will be appended to line
       * @param offset: the number of points skipped in segs.
       */
      static void append_segs_to_line(FASTMM::CORE::LineString *line,
                                      const FASTMM::CORE::LineString &segs,
                                      int offset = 0);
      Rtree rtree;             // Network rtree structure
      bool finalized = false;  // Flag to prevent modifications after finalization
      std::vector<Edge> edges; // all edges in the network
      NodeIDVec node_id_vec;
      unsigned int num_vertices;
      NodeIndexMap node_map;
      EdgeIndexMap edge_map;
      std::vector<FASTMM::CORE::Point> vertex_points;
      bool all_edges_have_speed = true; // True if all edges have speed, false if any edge lacks speed
    };
  }
}
#endif