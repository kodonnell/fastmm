//
// Created by Can Yang on 2020/3/22.
//

#include "mm/fmm/ubodt.hpp"
#include "util/util.hpp"

#include <fstream>
#include <stdexcept>

#ifdef BOOST_OS_WINDOWS
#include <boost/throw_exception.hpp>
#endif
#include <boost/format.hpp>
#include <boost/archive/binary_iarchive.hpp>

using namespace FASTMM;
using namespace FASTMM::CORE;
using namespace FASTMM::NETWORK;
using namespace FASTMM::MM;
UBODT::UBODT(int buckets_arg, int multiplier_arg) : buckets(buckets_arg), multiplier(multiplier_arg)
{
  SPDLOG_TRACE("Intialization UBODT with buckets {} multiplier {}",
               buckets, multiplier);
  hashtable = (Record **)malloc(sizeof(Record *) * buckets);
  for (int i = 0; i < buckets; i++)
  {
    hashtable[i] = nullptr;
  }
  SPDLOG_TRACE("Intialization UBODT finished");
}

UBODT::~UBODT()
{
  /* Clean hashtable */
  SPDLOG_TRACE("Clean UBODT");
  int i;
  for (i = 0; i < buckets; ++i)
  {
    Record *head = hashtable[i];
    Record *curr;
    while ((curr = head) != nullptr)
    {
      head = head->next;
      free(curr);
    }
  }
  // Destory hash table pointer
  free(hashtable);
  SPDLOG_TRACE("Clean UBODT finished");
}

Record *UBODT::look_up(NodeIndex source, NodeIndex target) const
{
  unsigned int h = cal_bucket_index(source, target);
  Record *r = hashtable[h];
  while (r != nullptr)
  {
    if (r->source == source && r->target == target)
    {
      return r;
    }
    else
    {
      r = r->next;
    }
  }
  return r;
}

std::vector<EdgeIndex> UBODT::look_sp_path(NodeIndex source,
                                           NodeIndex target) const
{
  std::vector<EdgeIndex> edges;
  if (source == target)
  {
    return edges;
  }
  Record *r = look_up(source, target);
  // No transition exist from source to target
  if (r == nullptr)
  {
    return edges;
  }
  while (r->first_n != target)
  {
    edges.push_back(r->next_e);
    r = look_up(r->first_n, target);
  }
  edges.push_back(r->next_e);
  return edges;
}

CompletePath UBODT::construct_complete_path(int traj_id, const TGOpath &path,
                                            const std::vector<Edge> &edges,
                                            std::vector<int> *indices,
                                            double reverse_tolerance) const
{
  CompletePath complete_path;
  if (!indices->empty())
    indices->clear();
  if (path.empty())
    return complete_path;
  int N = path.size();
  complete_path.push_back(path[0]->c->edge->id);
  int current_idx = 0;
  indices->push_back(current_idx);
  SPDLOG_TRACE("Insert index {}", current_idx);
  for (int i = 0; i < N - 1; ++i)
  {
    const Candidate *a = path[i]->c;
    const Candidate *b = path[i + 1]->c;

    SPDLOG_DEBUG("Check point {} a {} b {}", i, a->edge->id, b->edge->id);
    if ((a->edge->id != b->edge->id) || (a->offset - b->offset >
                                         a->edge->length * reverse_tolerance))
    {
      // segs stores edge index
      auto segs = look_sp_path(a->edge->target, b->edge->source);
      // No transition exist in UBODT
      if (segs.empty() && a->edge->target != b->edge->source)
      {
        SPDLOG_DEBUG("Edges not found connecting a b");
        SPDLOG_DEBUG("reverse movement {} tolerance {}",
                     a->offset - b->offset, a->edge->length * reverse_tolerance);
        SPDLOG_WARN("Traj {} unmatched as edge {} L {} offset {}"
                    " and edge {} L {} offset {} disconnected",
                    traj_id, a->edge->id, a->edge->length, a->offset,
                    b->edge->id, b->edge->length, b->offset);

        indices->clear();
        return CompletePath();
      }
      if (segs.empty())
      {
        SPDLOG_DEBUG("Edges ab are adjacent");
      }
      else
      {
        SPDLOG_DEBUG("Edges connecting ab are {}", segs);
      }
      for (int e : segs)
      {
        complete_path.push_back(edges[e].id);
        ++current_idx;
      }
      complete_path.push_back(b->edge->id);
      ++current_idx;
      indices->push_back(current_idx);
      SPDLOG_TRACE("Insert index {}", current_idx);
    }
    else
    {
      indices->push_back(current_idx);
      SPDLOG_TRACE("Insert index {}", current_idx);
    }
  }
  return complete_path;
}

double UBODT::get_delta() const
{
  return delta;
}

unsigned int UBODT::cal_bucket_index(NodeIndex source, NodeIndex target) const
{
  return (source * multiplier + target) % buckets;
}

void UBODT::insert(Record *r)
{
  // int h = (r->source*multiplier+r->target)%buckets ;
  int h = cal_bucket_index(r->source, r->target);
  r->next = hashtable[h];
  hashtable[h] = r;
  if (r->cost > delta)
    delta = r->cost;
  ++num_rows;
}

long UBODT::estimate_ubodt_rows(const std::string &filename)
{
  struct stat stat_buf;
  long rc = stat(filename.c_str(), &stat_buf);
  if (rc == 0)
  {
    int file_bytes = stat_buf.st_size;
    SPDLOG_TRACE("UBODT file size is {} bytes", file_bytes);
    std::string fn_extension = filename.substr(filename.find_last_of(".") + 1);
    std::transform(fn_extension.begin(),
                   fn_extension.end(),
                   fn_extension.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });
    int row_size = 28;
    return file_bytes / row_size;
  }
  return -1;
}

int UBODT::find_prime_number(double value)
{
  std::vector<int> prime_numbers = {
      5003, 10039, 20029, 50047, 100669, 200003, 500000,
      1000039, 2000083, 5000101, 10000103, 20000033};
  int N = prime_numbers.size();
  for (int i = 0; i < N; ++i)
  {
    if (value <= prime_numbers[i])
    {
      return prime_numbers[i];
    }
  }
  return prime_numbers[N - 1];
}

std::shared_ptr<UBODT> UBODT::read_ubodt_binary(const std::string &filename,
                                                int multiplier)
{
  SPDLOG_INFO("Reading UBODT file (binary format) from {}", filename);
  long rows = estimate_ubodt_rows(filename);
  int progress_step = 100000;
  SPDLOG_INFO("Estimated rows is {}", rows);
  int buckets = find_prime_number(rows / LOAD_FACTOR);
  SPDLOG_INFO("Number of buckets is {}", buckets);
  std::shared_ptr<UBODT> table = std::make_shared<UBODT>(buckets, multiplier);
  long NUM_ROWS = 0;
  std::ifstream ifs(filename.c_str(), std::ios::binary);
  // Check byte offset
  std::streampos archiveOffset = ifs.tellg();
  SPDLOG_INFO("Binary archive offset is {}", archiveOffset);
  std::streampos streamEnd = ifs.seekg(0, std::ios_base::end).tellg();
  SPDLOG_INFO("Binary file size is {} bytes", streamEnd);
  ifs.seekg(archiveOffset);
  boost::archive::binary_iarchive ia(ifs);
  SPDLOG_INFO("Start reading UBODT");
  while (true)
  {
    Record *r = (Record *)malloc(sizeof(Record));
    try
    {
      ia >> r->source;
      ia >> r->target;
      ia >> r->first_n;
      ia >> r->prev_n;
      ia >> r->next_e;
      ia >> r->cost;
      ++NUM_ROWS;
      r->next = nullptr;
      table->insert(r);
      if (NUM_ROWS % progress_step == 0)
      {
        SPDLOG_INFO("Read rows {}", NUM_ROWS);
      }
    }
    catch (...)
    {
      free(r);
      break;
    }
  }
  ifs.close();
  double lf = NUM_ROWS / (double)buckets;
  SPDLOG_INFO("Estimated load factor #elements/#tablebuckets {}", lf);
  if (lf > 10)
  {
    SPDLOG_WARN("Load factor is too large.");
  }
  SPDLOG_INFO("Finish reading UBODT with rows {}", NUM_ROWS);
  return table;
}
