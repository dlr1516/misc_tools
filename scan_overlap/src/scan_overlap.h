#ifndef SCAN_OVERLAP_H_
#define SCAN_OVERLAP_H_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/io.hpp>

namespace misc_tools {

using Vector2 = Eigen::Vector2d;

using Point = boost::geometry::model::d2::point_xy<double>;
using Linestring = boost::geometry::model::linestring<Point>;
using Polygon = boost::geometry::model::polygon<Point>;
using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;

double scan_overlap(const Polygon& p1, const Polygon& p2);

double scan_overlap(const std::vector<Vector2>& p1,
                    const std::vector<Vector2>& p2);

}  // namespace misc_tools

#endif