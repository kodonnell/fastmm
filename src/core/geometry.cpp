#include "core/geometry.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <iterator>
#include <vector>
#include <sstream>

std::ostream &FASTMM::CORE::operator<<(std::ostream &os,
                                    const FASTMM::CORE::LineString &rhs)
{
  os << std::setprecision(12) << boost::geometry::wkt(rhs.line);
  return os;
};

FASTMM::CORE::LineString FASTMM::CORE::wkt2linestring(const std::string &wkt)
{
  FASTMM::CORE::LineString line;
  boost::geometry::read_wkt(wkt, line.get_geometry());
  return line;
};
