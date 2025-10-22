/**
 * Fast map matching.
 *
 * ubodg_gen command line program
 *
 * @author: Can Yang
 * @version: 2020.01.31
 */

#ifndef FASTMM_SRC_MM_FASTMM_UBODT_GEN_ALGORITHM_HPP_
#define FASTMM_SRC_MM_FASTMM_UBODT_GEN_ALGORITHM_HPP_

#include "network/network.hpp"
#include "network/network_graph.hpp"

#ifdef BOOST_OS_WINDOWS
#include <boost/throw_exception.hpp>
#endif
#include <boost/archive/binary_oarchive.hpp>

namespace FASTMM
{
  namespace MM
  {
    class UBODTGenAlgorithm
    {
    public:
      UBODTGenAlgorithm(const NETWORK::Network &network,
                        const NETWORK::NetworkGraph &graph) : network_(network), ng_(graph) {
                                                              };
      void generate_ubodt(const std::string &filename, double delta) const;
      /**
       * Run precomputation in parallel and save result to a file
       * @param filename output file name
       * @param delta    upper bound value
       */
      void precompute_ubodt_omp(const std::string &filename, double delta) const;

    private:
      /**
       * Write the routing result to a binary stream
       * @param stream output binary stream
       * @param s      source node
       * @param pmap   predecessor map
       * @param dmap   distance map
       */
      void write_result_binary(boost::archive::binary_oarchive &stream,
                               NETWORK::NodeIndex s,
                               NETWORK::PredecessorMap &pmap,
                               NETWORK::DistanceMap &dmap) const;

      const NETWORK::Network &network_;
      const NETWORK::NetworkGraph &ng_;
    }; // UBODTGenAlgorithm
  }; // MM
}; // FASTMM

#endif // FASTMM_SRC_MM_FASTMM_UBODT_GEN_ALGORITHM_HPP_
