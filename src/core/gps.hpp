/**
 * Fast map matching.
 *
 * Definition of input trajectory format
 *
 * @author: Can Yang
 * @version: 2017.11.11
 */

#ifndef FMM_GPS_HPP
#define FMM_GPS_HPP

#include "core/geometry.hpp"

#include <vector>

namespace FMM
{

  namespace CORE
  {

    /**
     * %Trajectory class
     *
     * A GPS trajectory represented with id, geometry and timestamps
     */
    struct Trajectory
    {
      Trajectory() {};
      Trajectory(int id_arg, const LineString &geom_arg) : id(id_arg), geom(geom_arg) {};
      Trajectory(int id_arg, const LineString &geom_arg, const std::vector<double> &timestamps_arg)
          : id(id_arg), geom(geom_arg), timestamps(timestamps_arg)
      {
        if (geom.get_num_points() != static_cast<int>(timestamps.size()))
        {
          throw std::invalid_argument("Trajectory: timestamps and geometry must have the same length");
        }
        // Ensure timestamps are sorted
        for (size_t i = 1; i < timestamps.size(); ++i)
        {
          if (timestamps[i] < timestamps[i - 1])
          {
            throw std::invalid_argument("Trajectory: timestamps must be non-decreasing");
          }
        }
      }
      int id;                         /**< Id of the trajectory */
      LineString geom;                /**< Geometry of the trajectory */
      std::vector<double> timestamps; /**< Timestamps of the trajectory */

      // Return the number of points in the trajectory
      int size() const
      {
        return geom.get_num_points();
      }

      // Create a Trajectory from a vector of (x, y, t) tuples
      static Trajectory from_xy_tuples(int id, const std::vector<std::tuple<double, double>> &data)
      {
        LineString geom;
        for (const auto &item : data)
        {
          geom.add_point(Point(std::get<0>(item), std::get<1>(item)));
        }
        return Trajectory(id, geom);
      }

      // Create a Trajectory from a vector of (x, y, t) tuples
      static Trajectory from_xyt_tuples(int id, const std::vector<std::tuple<double, double, double>> &data)
      {
        LineString geom;
        std::vector<double> timestamps;
        for (const auto &item : data)
        {
          geom.add_point(Point(std::get<0>(item), std::get<1>(item)));
          timestamps.push_back(std::get<2>(item));
        }
        return Trajectory(id, geom, timestamps);
      }

      // Return as a vector of (x, y, t) tuples
      std::vector<std::tuple<double, double, double>> to_xyt_tuples() const
      {
        std::vector<std::tuple<double, double, double>> data;
        int n = geom.get_num_points();
        for (int i = 0; i < n; ++i)
        {
          double x = geom.get_x(i);
          double y = geom.get_y(i);
          double t = (i < timestamps.size()) ? timestamps[i] : 0.0;
          data.emplace_back(x, y, t);
        }
        return data;
      }

      // Return as a vector of (x, y, t) tuples
      std::vector<std::tuple<double, double>> to_xy_tuples() const
      {
        std::vector<std::tuple<double, double>> data;
        int n = geom.get_num_points();
        for (int i = 0; i < n; ++i)
        {
          double x = geom.get_x(i);
          double y = geom.get_y(i);
          data.emplace_back(x, y);
        }
        return data;
      }
    };

  }

}
#endif /* FMM_GPS_HPP */
