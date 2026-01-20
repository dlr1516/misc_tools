#include "scan_overlap.h"

namespace misc_tools {

double scan_overlap(const Polygon& p1, const Polygon& p2) {
    static int a = 0;
    MultiPolygon outputUnion;
    MultiPolygon outputIntersection;

    boost::geometry::intersection(p1, p2, outputIntersection);
    boost::geometry::union_(p1, p2, outputUnion);

    double areaIntersection = 0.0;
    /*for (const auto& poly : outputIntersection) {
        areaIntersection += boost::geometry::area(poly);
    }*/
    areaIntersection += boost::geometry::area(outputIntersection);
    double areaUnion = 0.0;
    /*for (const auto& poly : outputUnion) {
        areaUnion += boost::geometry::area(poly);
    }*/
    areaUnion += boost::geometry::area(outputUnion);
    std::cout << "union: " << areaUnion <<  " " << (boost::geometry::is_valid(outputUnion) ? "valid " : "non valid ") <<
        " inter: " << areaIntersection <<  " " << (boost::geometry::is_valid(outputIntersection) ? "valid " : "non valid ") << std::endl;

    {
        std::ofstream svg("./polys" + std::to_string(++a) + ".svg");
        boost::geometry::svg_mapper<Point> mapper(svg, 800, 800);
        mapper.add(p1);
        mapper.add(p2);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
    }
    {
        std::ofstream svg("./union" + std::to_string(a) + ".svg");
        boost::geometry::svg_mapper<Point> mapper(svg, 800, 800);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputUnion);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
        mapper.map(outputUnion, "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;stroke:rgb(0,0,0);stroke-width:3");
    }
    {
        std::ofstream svg("./inter" + std::to_string(a) + ".svg");
        boost::geometry::svg_mapper<Point> mapper(svg, 800, 800);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputIntersection);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
        mapper.map(outputIntersection, "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;stroke:rgb(0,0,0);stroke-width:3");
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
    auto p = scan1.front();
    boost::geometry::append(poly1, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));

    for (const auto& pt : scan2) {
        boost::geometry::append(poly2, Point(static_cast<double>(pt.x()),
                                            static_cast<double>(pt.y())));
    }
    p = scan2.front();
    boost::geometry::append(poly2, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));

    return scan_overlap(poly1, poly2);
}

}  // namespace misc_tools