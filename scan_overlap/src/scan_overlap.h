#ifndef SCAN_OVERLAP_H_
#define SCAN_OVERLAP_H_

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/io.hpp>

#include "transform_utils.h"

namespace misc_tools {

using Point = boost::geometry::model::d2::point_xy<double>;
using Linestring = boost::geometry::model::linestring<Point>;
using Polygon = boost::geometry::model::polygon<Point>;
using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;

double scan_overlap(const Polygon& p1, const Polygon& p2);

}  // namespace misc_tools

#endif