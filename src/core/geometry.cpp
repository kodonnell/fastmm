#include "core/geometry.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <iterator>
#include <vector>
#include <sstream>

std::ostream &FMM::CORE::operator<<(std::ostream &os,
                                    const FMM::CORE::LineString &rhs)
{
  os << std::setprecision(12) << boost::geometry::wkt(rhs.line);
  return os;
};

FMM::CORE::LineString FMM::CORE::wkt2linestring(const std::string &wkt)
{
  FMM::CORE::LineString line;
  boost::geometry::read_wkt(wkt, line.get_geometry());
  return line;
};
