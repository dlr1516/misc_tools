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

double scan_overlap_visualization(const Polygon& p1,
                                  const Polygon& p2,
                                  std::string dir) {
    MultiPolygon outputUnion;
    MultiPolygon outputIntersection;

    std::filesystem::path path(dir);
    if (!(std::filesystem::exists(path))) {
        std::cout << "Visualization directory doesn't Exists" << std::endl;

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

        mapper.map(p1,
                   "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);"
                   "stroke-width:2");
        mapper.map(p2,
                   "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);"
                   "stroke-width:2");
    }
    {
        std::ofstream svg(dir + "/union.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputUnion);

        mapper.map(p1,
                   "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);"
                   "stroke-width:2");
        mapper.map(p2,
                   "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);"
                   "stroke-width:2");
        mapper.map(outputUnion,
                   "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;"
                   "stroke:rgb(0,0,0);stroke-width:3");
    }
    {
        std::ofstream svg(dir + "/inter.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.add(p2);
        mapper.add(outputIntersection);

        mapper.map(p1,
                   "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);"
                   "stroke-width:2");
        mapper.map(p2,
                   "fill-opacity:0.5;fill:rgb(0,255,0);stroke:rgb(0,100,0);"
                   "stroke-width:2");
        mapper.map(outputIntersection,
                   "fill-opacity:0;fill:rgb(255,255,255);stroke-opacity:1;"
                   "stroke:rgb(0,0,0);stroke-width:3");
    }
    {
        std::ofstream svg(dir + "/poly1.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p1);
        mapper.map(p1,
                   "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);"
                   "stroke-width:2");
    }
    {
        std::ofstream svg(dir + "/poly2.svg");
        boost::geometry::svg_mapper<Point> mapper(svg, size, size);
        mapper.add(p2);
        mapper.map(p2,
                   "fill-opacity:0.5;fill:rgb(255,0,0);stroke:rgb(100,0,0);"
                   "stroke-width:2");
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

    //std::ofstream filePoly1("poly1.txt");

    for (const auto& pt : scan1) {
        boost::geometry::append(poly1, Point(static_cast<double>(pt.x()),
                                            static_cast<double>(pt.y())));
        //filePoly1 << poly1.outer().back().x() << " " << poly1.outer().back().y() << "\n";
    }
    auto p = scan1.front();
    boost::geometry::append(poly1, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));
    /*filePoly1 << poly1.outer().back().x() << " " << poly1.outer().back().y() << "\n";
    filePoly1.close(); 

    std::ofstream filePoly2("poly2.txt");*/

    for (const auto& pt : scan2) {
        boost::geometry::append(poly2, Point(static_cast<double>(pt.x()),
                                            static_cast<double>(pt.y())));
        //filePoly2 << poly2.outer().back().x() << " " << poly2.outer().back().y() << "\n";
    }
    p = scan2.front();
    boost::geometry::append(poly2, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));
    /*filePoly2 << poly2.outer().back().x() << " " << poly2.outer().back().y() << "\n";
    filePoly2.close(); */

    return scan_overlap(poly1, poly2);
}

double scan_overlap_visualization(const std::vector<Vector2>& scan1,
                                  const std::vector<Vector2>& scan2,
                                  std::string dir) {
    Polygon poly1, poly2;

    for (const auto& pt : scan1) {
        boost::geometry::append(poly1, Point(static_cast<double>(pt.x()),
                                             static_cast<double>(pt.y())));
    }
    auto p = scan1.front();
    boost::geometry::append(
        poly1, Point(static_cast<double>(p.x()), static_cast<double>(p.y())));

    for (const auto& pt : scan2) {
        boost::geometry::append(poly2, Point(static_cast<double>(pt.x()),
                                             static_cast<double>(pt.y())));
    }
    p = scan2.front();
    boost::geometry::append(
        poly2, Point(static_cast<double>(p.x()), static_cast<double>(p.y())));

    return scan_overlap_visualization(poly1, poly2, dir);
}

#if BOOST_VERSION >= 174000
void draw_graph(const Graph& graph, const std::vector<Cloud>& clouds) {
    std::function<std::array<double, 3>(double)> cmap =
        boost::math::tools::viridis<double>;
    double invSize = 1. / clouds.size();
    std::vector<Polygon> polys;
    std::vector<std::array<uint8_t, 4UL>> colors;

    std::ofstream svg("graph.svg");
    boost::geometry::svg_mapper<Point> mapper(svg, 800, 800);
    for (int i = 0; i < clouds.size(); i++) {
        Cloud cloud = clouds[i];
        std::array<uint8_t, 4UL> c =
            boost::math::tools::to_8bit_rgba(cmap(i * invSize));
        Polygon poly;

        for (const auto& p : cloud)
            boost::geometry::append(poly, Point(static_cast<double>(p.x()),
                                                static_cast<double>(p.y())));
        auto p = cloud.front();
        boost::geometry::append(poly, Point(static_cast<double>(p.x()),
                                            static_cast<double>(p.y())));
        polys.push_back(poly);
        colors.push_back(c);
        mapper.add(poly);
    }

    for (int i = 0; i < polys.size(); i++) {
        Node n = graph[i];
        auto c = colors[i];
        auto p = polys[i];
        std::string colorF = "(" + std::to_string(c[0]) + "," +
                             std::to_string(c[1]) + "," + std::to_string(c[2]) +
                             ")";
        std::string colorS = "(" + std::to_string((int)c[0] * .75) + "," +
                             std::to_string((int)c[1] * .75) + "," +
                             std::to_string((int)c[2] * .75) + ")";
        mapper.map(p, "fill-opacity:0.5;fill:rgb" + colorF + ";stroke:rgb" +
                          colorS + ";stroke-width:2");
        mapper.text(Point(.0, .0), std::to_string(n.id),
                    "fill:rgb" + colorF + ";fill-opacity:1;stroke:none", .0,
                    i * 15);
    }
}
#else
void draw_graph(const Graph& graph, const std::vector<Cloud>& clouds) {}
#endif

}  // namespace misc_tools