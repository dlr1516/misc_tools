#ifndef SCAN_OVERLAP_H_
#define SCAN_OVERLAP_H_

#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/io/wkt/wkt.hpp>

#if BOOST_VERSION >= 174000
#include <boost/math/tools/color_maps.hpp>
#endif

#include <filesystem>

#include "transform_utils.h"

namespace misc_tools {

using Point = boost::geometry::model::d2::point_xy<double>;
using Linestring = boost::geometry::model::linestring<Point>;
using Polygon = boost::geometry::model::polygon<Point, false, true>;
using MultiPolygon = boost::geometry::model::multi_polygon<Polygon>;

double scan_overlap(const Polygon& p1, const Polygon& p2);
double scan_overlap_visualization(const Polygon& p1,
                                  const Polygon& p2,
                                  std::string dir);

double scan_overlap(const std::vector<Vector2>& p1,
                    const std::vector<Vector2>& p2);
double scan_overlap_visualization(const std::vector<Vector2>& p1,
                                  const std::vector<Vector2>& p2,
                                  std::string dir);

void draw_graph(const Graph& graph, const std::vector<Cloud>& clouds);

}  // namespace misc_tools

#endif