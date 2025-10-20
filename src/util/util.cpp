#include "util/util.hpp"

#include <iostream>
#include <string>
#ifdef _WIN32
#include <sys/types.h>
#include <sys/stat.h>
#else
#include <sys/stat.h>
#endif
#include <chrono>
#include <vector>
#include <ctime>
#include "mm/mm_type.hpp"

#if defined BOOST_OS_WINDOWS && !defined S_ISDIR
#define S_ISDIR(m) (((m) & _S_IFDIR) == _S_IFDIR)
#endif

#ifdef BOOST_NO_EXCEPTIONS

namespace boost
{

  BOOST_NORETURN void throw_exception(const std::exception &e)
  {
    throw e;
  }
  BOOST_NORETURN void throw_exception(const std::exception &e, boost::source_location const &loc)
  {
    throw e;
  }

} // namespace boost

#endif // BOOST_NO_EXCEPTIONS

namespace std
{

  std::ostream &operator<<(std::ostream &os,
                           const FMM::MM::TrajectoryCandidates &tr_cs)
  {
    os << "\nCandidate "
       << std::fixed << std::setw(4) << "step" << ";"
       << std::fixed << std::setw(6) << "index" << ";"
       << std::fixed << std::setw(8) << "offset" << ";"
       << std::fixed << std::setw(8) << "distance" << ";"
       << std::fixed << std::setw(8) << "edge_id" << '\n';
    for (auto tr_cs_iter = tr_cs.begin();
         tr_cs_iter != tr_cs.end(); ++tr_cs_iter)
    {
      for (auto p_cs_iter = tr_cs_iter->begin();
           p_cs_iter != tr_cs_iter->end();
           ++p_cs_iter)
      {
        os << "Candidate "
           << std::fixed << std::setw(4) << std::distance(tr_cs.begin(), tr_cs_iter) << ";"
           << std::fixed << std::setw(6) << p_cs_iter->index << ";"
           << std::fixed << std::setw(8) << p_cs_iter->offset << ";"
           << std::fixed << std::setw(8) << p_cs_iter->dist << ";"
           << std::fixed << std::setw(8) << p_cs_iter->edge->id << '\n';
      }
    }
    return os;
  }

  std::ostream &operator<<(std::ostream &os,
                           const FMM::MM::OptCandidatePath &optimal_path)
  {
    for (int i = 0; i < optimal_path.size(); ++i)
    {
      // std::cout <<"Write edge "<< i <<" edge "<< optimal_path[i]->edge->id <<"\n";
      os << optimal_path[i]->edge->id;
      if (i != optimal_path.size() - 1)
        os << ",";
    }
    return os;
  }

  std::ostream &operator<<(std::ostream &os,
                           const FMM::CORE::Point &geom)
  {
    os << std::setprecision(12) << boost::geometry::wkt(geom);
    return os;
  }

} // namespace std

namespace FMM
{

  namespace UTIL
  {

    TimePoint get_current_time()
    {
      return std::chrono::system_clock::now();
    };

    double get_duration(const TimePoint &t1, const TimePoint &t2)
    {
      return std::chrono::duration_cast<
                 std::chrono::milliseconds>(t2 - t1)
                 .count() /
             1000.;
    };

    // Print a timestamp
    void print_time(
        const TimePoint &timestamp)
    {
      std::time_t timestamp_t = std::chrono::system_clock::to_time_t(timestamp);
      std::cout << "Time " << std::ctime(&timestamp_t) << '\n';
    };

    bool file_exists(const char *filename)
    {
#ifdef _WIN32
      struct _stat buf;
      if (_stat(filename, &buf) != -1)
      {
        return true;
      }
#else
      struct stat buf;
      if (stat(filename, &buf) != -1)
      {
        return true;
      }
#endif
      return false;
    }

    bool file_exists(const std::string &filename)
    {
      return file_exists(filename.c_str());
    }

    std::vector<std::string> split_string(const std::string &str)
    {
      char delim = ',';
      std::vector<std::string> result;
      std::stringstream ss(str);
      std::string intermediate;
      while (getline(ss, intermediate, delim))
      {
        result.push_back(intermediate);
      }
      return result;
    }

    bool string2bool(const std::string &str)
    {
      return str == "true" || str == "t" || str == "1";
    }

    bool check_file_extension(const std::string &filename,
                              const std::string &extension_list_str)
    {
      bool result = false;
      std::stringstream ss;
      std::string fn_extension = filename.substr(
          filename.find_last_of('.') + 1);
      std::vector<std::string> extensions =
          split_string(extension_list_str);
      for (const auto &extension : extensions)
      {
        if (fn_extension == extension)
          result = true;
      }
      return result;
    }

    bool folder_exist(const std::string &folder_name)
    {
      if (folder_name.empty())
        return true;
#ifdef _WIN32
      struct _stat sb;
      if (_stat(folder_name.c_str(), &sb) == 0 && (sb.st_mode & _S_IFDIR))
      {
        return true;
      }
#else
      struct stat sb;
      if (stat(folder_name.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
      {
        return true;
      }
#endif
      return false;
    }

    std::string get_file_directory(const std::string &fn)
    {
      std::size_t found = fn.find_last_of("/");
      if (found != std::string::npos)
      {
        return fn.substr(0, found);
      }
      return {};
    };

    std::istream &safe_get_line(std::istream &is, std::string &t, char delim)
    {
      t.clear();
      std::istream::sentry se(is, true);
      std::streambuf *sb = is.rdbuf();
      for (;;)
      {
        int c = sb->sbumpc();
        switch (c)
        {
        case '\n':
          return is;
        case '\r':
          if (sb->sgetc() == '\n')
            sb->sbumpc();
          return is;
        case std::streambuf::traits_type::eof():
          // Also handle the case when the last line has no line ending
          if (t.empty())
            is.setstate(std::ios::eofbit);
          return is;
        default:
          if (c == delim)
          {
            return is;
          }
          t += (char)c;
        }
      }
    }

  } // Util
} // FMM
