#include "scan_overlap.h"

namespace misc_tools {

double scan_overlap(const Polygon& p1, const Polygon& p2) {
    MultiPolygon outputUnion;
    MultiPolygon outputIntersection;

    boost::geometry::intersection(p1, p2, outputIntersection);
    boost::geometry::union_(p1, p2, outputUnion);

    double areaIntersection = boost::geometry::area(outputIntersection);
    double areaUnion = boost::geometry::area(outputUnion);
    
    if (areaUnion > 0.0) {
        return areaIntersection / areaUnion;
    }
    return 0.0;
}

double scan_overlap_visualization(const Polygon& p1, const Polygon& p2, std::string dir) {
    MultiPolygon outputUnion;
    MultiPolygon outputIntersection;

    std::filesystem::path path(dir);
    if(!(std::filesystem::exists(path))){
        std::cout<<"Visualization directory doesn't Exists"<<std::endl;

        if (std::filesystem::create_directories(path))
            std::cout << "....Successfully Created !" << std::endl;
    }

    boost::geometry::intersection(p1, p2, outputIntersection);
    boost::geometry::union_(p1, p2, outputUnion);

    double areaIntersection = boost::geometry::area(outputIntersection);
    double areaUnion = boost::geometry::area(outputUnion);
    int size = 400;

    {
        std::ofstream svg(dir + "/polys.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.add(p2);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
    }
    {
        std::ofstream svg(dir + "/union.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputUnion);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
        mapper.map(outputUnion, "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;stroke:rgb(0,0,0);stroke-width:3");
    }
    {
        std::ofstream svg(dir + "/inter.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputIntersection);

        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);stroke-width:2");
        mapper.map(outputIntersection, "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;stroke:rgb(0,0,0);stroke-width:3");
    }
    {
        std::ofstream svg(dir + "/poly1.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.map(p1, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
    }
    {
        std::ofstream svg(dir + "/poly2.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p2);
        mapper.map(p2, "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);stroke-width:2");
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

    std::ofstream filePoly1("poly1.txt");

    for (const auto& pt : scan1) {
        boost::geometry::append(poly1, Point(static_cast<double>(pt.x()),
                                            static_cast<double>(pt.y())));
        filePoly1 << poly1.outer().back().x() << " " << poly1.outer().back().y() << "\n";
    }
    auto p = scan1.front();
    boost::geometry::append(poly1, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));
    filePoly1 << poly1.outer().back().x() << " " << poly1.outer().back().y() << "\n";
    filePoly1.close(); 

    std::ofstream filePoly2("poly2.txt");

    for (const auto& pt : scan2) {
        boost::geometry::append(poly2, Point(static_cast<double>(pt.x()),
                                            static_cast<double>(pt.y())));
        filePoly2 << poly2.outer().back().x() << " " << poly2.outer().back().y() << "\n";
    }
    p = scan2.front();
    boost::geometry::append(poly2, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));
    filePoly2 << poly2.outer().back().x() << " " << poly2.outer().back().y() << "\n";
    filePoly2.close(); 

    return scan_overlap(poly1, poly2);
}

double scan_overlap_visualization(const std::vector<Vector2>& scan1,
                    const std::vector<Vector2>& scan2, std::string dir) {
    Polygon poly1, poly2;

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

    return scan_overlap_visualization(poly1, poly2, dir);
}

}  // namespace misc_tools