#include "scan_overlap.h"

namespace misc_tools {

double scan_overlap(const Polygon& p1, const Polygon& p2) {
    std::vector<Polygon> outputUnion;
    std::vector<Polygon> outputIntersection;

    boost::geometry::intersection(p1, p2, outputIntersection);
    boost::geometry::union_(p1, p2, outputUnion);

    double areaIntersection = 0.0;
    for (const auto& poly : outputIntersection) {
        areaIntersection += boost::geometry::area(poly);
    }

    double areaUnion = 0.0;
    for (const auto& poly : outputUnion) {
        areaUnion += boost::geometry::area(poly);
    }

    if (areaUnion > 0.0) {
        return areaIntersection / areaUnion;
    }
    return 0.0;
}

double scan_overlap(const std::vector<Vector2>& scan1,
                    const std::vector<Vector2>& scan2) {
    Polygon poly1, poly2;
    // boost::geometry::append(poly1, scan1);
    // boost::geometry::append(poly2, scan2);

    for (const auto& pt : scan1) {
        boost::geometry::append(poly1, Point(static_cast<double>(pt.x()),
                                             static_cast<double>(pt.y())));
    }

    for (const auto& pt : scan2) {
        boost::geometry::append(poly2, Point(static_cast<double>(pt.x()),
                                             static_cast<double>(pt.y())));
    }

    return scan_overlap(poly1, poly2);
}

}  // namespace misc_tools