#include "scan_overlap"

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

}  // namespace misc_tools