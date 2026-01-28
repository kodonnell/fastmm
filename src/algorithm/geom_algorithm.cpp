#include "algorithm/geom_algorithm.hpp"
#include "util/debug.hpp"

#include <cmath>
#include <cstdlib>

std::vector<double> FASTMM::ALGORITHM::calculate_linestring_euclidean_distances(
    const FASTMM::CORE::LineString &trajectory)
{
  int N = trajectory.get_num_points();
  std::vector<double> lengths(N - 1);
  double x0 = trajectory.get_x(0);
  double y0 = trajectory.get_y(0);
  for (int i = 1; i < N; ++i)
  {
    double x1 = trajectory.get_x(i);
    double y1 = trajectory.get_y(i);
    double dx = x1 - x0;
    double dy = y1 - y0;
    lengths[i - 1] = sqrt(dx * dx + dy * dy);
    x0 = x1;
    y0 = y1;
  }
  return lengths;
}

void FASTMM::ALGORITHM::append_segs_to_line(
    FASTMM::CORE::LineString *line, const FASTMM::CORE::LineString &segs,
    int offset)
{
  int Npoints = segs.get_num_points();
  for (int i = 0; i < Npoints; ++i)
  {
    if (i >= offset)
    {
      line->add_point(segs.get_x(i), segs.get_y(i));
    }
  }
}

FASTMM::CORE::LineString FASTMM::ALGORITHM::reverse_geometry(
    const FASTMM::CORE::LineString &rhs)
{
  FASTMM::CORE::LineString line;
  int Npoints = rhs.get_num_points();
  for (int i = Npoints - 1; i >= 0; --i)
  {
    line.add_point(rhs.get_x(i), rhs.get_y(i));
  }
  return line;
}

std::vector<FASTMM::CORE::LineString> FASTMM::ALGORITHM::split_line(
    const FASTMM::CORE::LineString &line, double delta)
{
  std::vector<FASTMM::CORE::LineString> segments;
  if (line.get_length() <= delta)
  {
    segments.push_back(line);
    return segments;
  }
  int Npoints = line.get_num_points();
  if (Npoints < 2)
  {
    return segments;
  }
  double length_visited = 0;
  FASTMM::CORE::LineString interpolated_line;
  int i = 1; // Point in line
  double cx = line.get_x(0);
  double cy = line.get_y(0);
  interpolated_line.add_point(cx, cy);
  while (i < Npoints)
  {
    double nx = line.get_x(i);
    double ny = line.get_y(i);
    double temp = std::sqrt((nx - cx) * (nx - cx) + (ny - cy) * (ny - cy));
    if (length_visited <= delta && length_visited + temp > delta)
    {
      // Split the edge
      double ratio = (delta - length_visited) / temp;
      cx = ratio * (nx - cx) + cx;
      cy = ratio * (ny - cy) + cy;
      interpolated_line.add_point(cx, cy);
      segments.push_back(interpolated_line);
      interpolated_line.clear();
      interpolated_line.add_point(cx, cy);
      length_visited = 0;
    }
    else
    {
      cx = nx;
      cy = ny;
      interpolated_line.add_point(cx, cy);
      length_visited += temp;
      ++i;
    }
  }
  segments.push_back(interpolated_line);
  return segments;
}

FASTMM::CORE::LineString FASTMM::ALGORITHM::interpolate_line_distances(
    const FASTMM::CORE::LineString &line, const std::vector<double> &distances)
{
  FASTMM::CORE::LineString interpolated_line;
  int Npoints = line.get_num_points();
  int distance_count = distances.size();
  double length_visited = 0;
  int i = 0; // Point in line
  int j = 0; // Element visited in distances vector
  while (i < Npoints - 1 && j < distance_count)
  {
    double x1 = line.get_x(i);
    double y1 = line.get_y(i);
    double x2 = line.get_x(i + 1);
    double y2 = line.get_y(i + 1);
    double temp = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    while (j < distance_count && length_visited <= distances[j] && length_visited + temp > distances[j])
    {
      double ratio = (distances[j] - length_visited) / temp;
      // SPDLOG_INFO("Ratio is {}",ratio);
      interpolated_line.add_point(ratio * (x2 - x1) + x1,
                                  ratio * (y2 - y1) + y1);
      ++j;
    }
    length_visited += temp;
    ++i;
  }
  if (length_visited < distances.back())
  {
    interpolated_line.add_point(line.get_x(Npoints - 1),
                                line.get_y(Npoints - 1));
  }
  return interpolated_line;
}

FASTMM::CORE::LineString FASTMM::ALGORITHM::interpolate_line_distance(
    const FASTMM::CORE::LineString &line, double distance)
{
  double length = line.get_length();
  int k = (int)ceil(length / distance);
  std::vector<double> distances;
  for (int i = 0; i < k + 1; ++i)
  {
    distances.push_back(distance * i);
  }
  return interpolate_line_distances(line, distances);
}

FASTMM::CORE::LineString FASTMM::ALGORITHM::interpolate_line_kpoints(
    const FASTMM::CORE::LineString &line, int k)
{
  double length = line.get_length();
  std::vector<double> distances;
  for (int i = 0; i < k; ++i)
  {
    distances.push_back((double)rand() / RAND_MAX * length);
  }
  std::sort(distances.begin(), distances.end());
  return interpolate_line_distances(line, distances);
}

void FASTMM::ALGORITHM::boundingbox_geometry(
    const FASTMM::CORE::LineString &linestring, double *x1, double *y1,
    double *x2, double *y2)
{
  int Npoints = linestring.get_num_points();
  *x1 = DBL_MAX;
  *y1 = DBL_MAX;
  *x2 = DBL_MIN;
  *y2 = DBL_MIN;
  double x, y;
  for (int i = 0; i < Npoints; ++i)
  {
    x = linestring.get_x(i);
    y = linestring.get_y(i);
    if (x < *x1)
      *x1 = x;
    if (y < *y1)
      *y1 = y;
    if (x > *x2)
      *x2 = x;
    if (y > *y2)
      *y2 = y;
  }
}

std::vector<double> FASTMM::ALGORITHM::calc_length_to_end_vec(
    const FASTMM::CORE::LineString &geom)
{
  int N = geom.get_num_points();
  if (N < 2)
    return std::vector<double>();
  std::vector<double> result(N - 1);
  double x0 = geom.get_x(0);
  double y0 = geom.get_y(0);
  for (int i = 1; i < N; ++i)
  {
    double x1 = geom.get_x(i);
    double y1 = geom.get_y(i);
    double dx = x1 - x0;
    double dy = y1 - y0;
    result[i - 1] = sqrt(dx * dx + dy * dy);
    x0 = x1;
    y0 = y1;
  }
  double temp = 0;
  for (int i = N - 2; i >= 0; --i)
  {
    result[i] = temp + result[i];
    temp = result[i];
  }
  return result;
}

void FASTMM::ALGORITHM::closest_point_on_segment(
    double x, double y, double x1, double y1, double x2, double y2,
    double *dist, double *offset)
{
  double L2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (L2 == 0.0)
  {
    *dist = std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    *offset = 0.0;
    return;
  }
  double x1_x = x - x1;
  double y1_y = y - y1;
  double x1_x2 = x2 - x1;
  double y1_y2 = y2 - y1;
  double ratio = (x1_x * x1_x2 + y1_y * y1_y2) / L2;
  ratio = (ratio > 1) ? 1 : ratio;
  ratio = (ratio < 0) ? 0 : ratio;
  double prj_x = x1 + ratio * (x1_x2);
  double prj_y = y1 + ratio * (y1_y2);
  *offset = std::sqrt((prj_x - x1) * (prj_x - x1) +
                      (prj_y - y1) * (prj_y - y1));
  *dist = std::sqrt((prj_x - x) * (prj_x - x) + (prj_y - y) * (prj_y - y));
} // closest_point_on_segment

void FASTMM::ALGORITHM::closest_point_on_segment(
    double x, double y, double x1, double y1,
    double x2, double y2, double *dist,
    double *offset, double *closest_x,
    double *closest_y)
{
  double L2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (L2 == 0.0)
  {
    *dist = std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
    *offset = 0.0;
    *closest_x = x1;
    *closest_y = y1;
    return;
  }
  double x1_x = x - x1;
  double y1_y = y - y1;
  double x1_x2 = x2 - x1;
  double y1_y2 = y2 - y1;
  double ratio = (x1_x * x1_x2 + y1_y * y1_y2) / L2;
  ratio = (ratio > 1) ? 1 : ratio;
  ratio = (ratio < 0) ? 0 : ratio;
  double prj_x = x1 + ratio * (x1_x2);
  double prj_y = y1 + ratio * (y1_y2);
  *offset = std::sqrt((prj_x - x1) * (prj_x - x1) +
                      (prj_y - y1) * (prj_y - y1));
  *dist = std::sqrt((prj_x - x) * (prj_x - x) + (prj_y - y) * (prj_y - y));
  *closest_x = prj_x;
  *closest_y = prj_y;
} // closest_point_on_segment

void FASTMM::ALGORITHM::linear_referencing(
    double px, double py, const FASTMM::CORE::LineString &linestring,
    double *result_dist, double *result_offset)
{
  int Npoints = linestring.get_num_points();
  double min_dist = DBL_MAX;
  double final_offset = DBL_MAX;
  double length_parsed = 0;
  int i = 0;
  // Iterating to check p(i) == p(i+2)
  while (i < Npoints - 1)
  {
    double x1 = linestring.get_x(i);
    double y1 = linestring.get_y(i);
    double x2 = linestring.get_x(i + 1);
    double y2 = linestring.get_y(i + 1);
    double temp_min_dist;
    double temp_min_offset;
    closest_point_on_segment(px, py, x1, y1, x2, y2,
                             &temp_min_dist, &temp_min_offset);
    if (temp_min_dist < min_dist)
    {
      min_dist = temp_min_dist;
      final_offset = length_parsed + temp_min_offset;
    }
    length_parsed += std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    ++i;
  }
  *result_dist = min_dist;
  *result_offset = final_offset;
} // linear_referencing

void FASTMM::ALGORITHM::linear_referencing(
    double px, double py, const FASTMM::CORE::LineString &linestring,
    double *result_dist, double *result_offset,
    double *proj_x, double *proj_y)
{
  int Npoints = linestring.get_num_points();
  double min_dist = DBL_MAX;
  double temp_x = 0, temp_y = 0;
  double final_offset = DBL_MAX;
  double length_parsed = 0;
  int i = 0;
  // Iterating to check p(i) == p(i+2)
  while (i < Npoints - 1)
  {
    double x1 = linestring.get_x(i);
    double y1 = linestring.get_y(i);
    double x2 = linestring.get_x(i + 1);
    double y2 = linestring.get_y(i + 1);
    double temp_min_dist;
    double temp_min_offset;
    closest_point_on_segment(px, py, x1, y1, x2, y2,
                             &temp_min_dist, &temp_min_offset,
                             &temp_x, &temp_y);
    if (temp_min_dist < min_dist)
    {
      min_dist = temp_min_dist;
      final_offset = length_parsed + temp_min_offset;
      *proj_x = temp_x;
      *proj_y = temp_y;
    }
    length_parsed += std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    ++i;
  }
  *result_dist = min_dist;
  *result_offset = final_offset;
} // linear_referencing

void FASTMM::ALGORITHM::locate_point_by_offset(
    const FASTMM::CORE::LineString &linestring, double offset,
    double *x, double *y)
{
  int Npoints = linestring.get_num_points();
  if (offset <= 0.0)
  {
    *x = linestring.get_x(0);
    *y = linestring.get_y(0);
    return;
  }
  double L_processed = 0; // length parsed
  int i = 0;
  double px = 0;
  double py = 0;
  bool found = false;
  // Find the idx of the point to be exported close to p
  while (i < Npoints - 1)
  {
    double x1 = linestring.get_x(i);
    double y1 = linestring.get_y(i);
    double x2 = linestring.get_x(i + 1);
    double y2 = linestring.get_y(i + 1);
    double deltaL = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double ratio = (offset - L_processed) / deltaL;
    if (offset >= L_processed && offset <= L_processed + deltaL)
    {
      px = x1 + ratio * (x2 - x1);
      py = y1 + ratio * (y2 - y1);
      found = true;
      break;
    }
    ++i;
    L_processed += deltaL;
  }
  if (found)
  {
    *x = px;
    *y = py;
  }
  else
  {
    *x = linestring.get_x(Npoints - 1);
    *y = linestring.get_y(Npoints - 1);
  }
} // calculate_offset_point

FASTMM::CORE::LineString FASTMM::ALGORITHM::cutoffseg_unique(
    const FASTMM::CORE::LineString &linestring, double offset1, double offset2)
{
  // If offset1 > offset2, swap them and reverse the result at the end
  bool needs_reversal = (offset1 > offset2);
  if (needs_reversal)
  {
    std::swap(offset1, offset2);
  }

  FASTMM::CORE::LineString cutoffline;
  int Npoints = linestring.get_num_points();
  if (Npoints == 2)
  {
    // A single segment
    double x1 = linestring.get_x(0);
    double y1 = linestring.get_y(0);
    double x2 = linestring.get_x(1);
    double y2 = linestring.get_y(1);
    double L = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double ratio1 = offset1 / L;
    double new_x1 = x1 + ratio1 * (x2 - x1);
    double new_y1 = y1 + ratio1 * (y2 - y1);
    double ratio2 = offset2 / L;
    double new_x2 = x1 + ratio2 * (x2 - x1);
    double new_y2 = y1 + ratio2 * (y2 - y1);
    cutoffline.add_point(new_x1, new_y1);
    cutoffline.add_point(new_x2, new_y2);
  }
  else
  {
    // Multiple segments
    double l1 = 0;
    double l2 = 0;
    int i = 0;
    while (i < Npoints - 1)
    {
      double x1 = linestring.get_x(i);
      double y1 = linestring.get_y(i);
      double x2 = linestring.get_x(i + 1);
      double y2 = linestring.get_y(i + 1);
      double deltaL = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      l2 = l1 + deltaL;
      // Insert p1
      if (l1 >= offset1 && l1 <= offset2)
      {
        cutoffline.add_point(x1, y1);
      }

      // Insert p between p1 and p2
      if (offset1 > l1 && offset1 < l2)
      {
        double ratio1 = (offset1 - l1) / deltaL;
        double px = x1 + ratio1 * (x2 - x1);
        double py = y1 + ratio1 * (y2 - y1);
        cutoffline.add_point(px, py);
      }

      if (offset2 > l1 && offset2 < l2)
      {
        double ratio2 = (offset2 - l1) / deltaL;
        double px = x1 + ratio2 * (x2 - x1);
        double py = y1 + ratio2 * (y2 - y1);
        cutoffline.add_point(px, py);
      }

      // last point
      if (i == Npoints - 2 && offset2 >= l2)
      {
        cutoffline.add_point(x2, y2);
      }
      l1 = l2;
      ++i;
    }
  }

  // Reverse if offsets were swapped
  if (needs_reversal)
  {
    boost::geometry::reverse(cutoffline.get_geometry());
  }

  return cutoffline;
} // cutoffseg_twoparameters

FASTMM::CORE::LineString FASTMM::ALGORITHM::cutoffseg(
    const FASTMM::CORE::LineString &linestring, double offset, int mode)
{
  double L = linestring.get_length();
  double offset1, offset2;
  if (mode == 0)
  {
    offset1 = offset;
    offset2 = L;
  }
  else
  {
    offset1 = 0;
    offset2 = offset;
  }
  return FASTMM::ALGORITHM::cutoffseg_unique(linestring, offset1, offset2);
}
