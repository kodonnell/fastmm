//
// Created by Can Yang on 2020/4/1.
//

#include "mm/fmm/ubodt_gen_algorithm.hpp"
#include "mm/fmm/ubodt.hpp"

#include "util/debug.hpp"
#include <fstream>
#include <omp.h>

using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::NETWORK;
using namespace FASTMM::MM;

void UBODTGenAlgorithm::generate_ubodt(const std::string &filename, double delta, const std::string &network_hash) const
{
  std::ostringstream oss;
  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  precompute_ubodt_omp(filename, delta, network_hash);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  double time_spent = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.;
  SPDLOG_INFO("Status: success");
  SPDLOG_INFO("Time takes {} seconds", time_spent);
};

void UBODTGenAlgorithm::precompute_ubodt_omp(const std::string &filename, double delta, const std::string &network_hash) const
{
  int num_vertices = ng_.get_num_vertices();
  int step_size = num_vertices / 10;
  if (step_size < 10)
    step_size = 10;
  std::ofstream myfile(filename, std::ios::binary);
  SPDLOG_INFO("Start to generate UBODT with delta {}", delta);

  boost::archive::binary_oarchive oa(myfile);

  // Write metadata header
  int version = 1;
  int mode_int = static_cast<int>(mode_);
  int multiplier = num_vertices + 1; // Auto-computed: num_vertices + 1
  oa << version << mode_int << delta << num_vertices << multiplier << network_hash;
  SPDLOG_INFO("UBODT metadata: version={}, mode={}, delta={}, num_vertices={}, multiplier={}, hash={}",
              version, mode_int, delta, num_vertices, multiplier, network_hash);

  int progress = 0;
#pragma omp parallel
  {
#pragma omp for
    for (int source = 0; source < num_vertices; ++source)
    {
      ++progress;
      if (progress % step_size == 0)
      {
        SPDLOG_INFO("Progress {} / {}", progress, num_vertices);
      }
      PredecessorMap pmap;
      DistanceMap dmap;
      std::stringstream node_output_buf;
      ng_.single_source_upperbound_dijkstra(source, delta, &pmap, &dmap);
      write_result_binary(oa, source, pmap, dmap);
    }
  }
  myfile.close();
}

/**
 * Write the result of routing from a single source node in
 * binary format
 *
 * @param stream output stream
 * @param s      source node
 * @param pmap   predecessor map
 * @param dmap   distance map
 */
void UBODTGenAlgorithm::write_result_binary(boost::archive::binary_oarchive &stream,
                                            NodeIndex s,
                                            PredecessorMap &pmap,
                                            DistanceMap &dmap) const
{
  std::vector<Record> source_map;
  for (auto iter = pmap.begin(); iter != pmap.end(); ++iter)
  {
    NodeIndex cur_node = iter->first;
    if (cur_node != s)
    {
      NodeIndex prev_node = iter->second;
      NodeIndex v = cur_node;
      NodeIndex u;
      // When u=s, v is the next node visited
      while ((u = pmap[v]) != s)
      {
        v = u;
      }
      NodeIndex successor = v;
      // Write the result
      double cost = dmap[successor];
      EdgeIndex edge_index = ng_.get_edge_index(s, successor, cost);
      source_map.push_back(
          {s,
           cur_node,
           successor,
           prev_node,
           edge_index,
           dmap[cur_node]});
    }
  }
#pragma omp critical
  for (Record &r : source_map)
  {
    stream << r.source << r.target << r.first_n << r.prev_n << r.next_e << r.cost;
  }
}
