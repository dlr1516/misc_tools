#include <iostream>
#include <vector>

#include <rofl/common/param_map.h>

#include "transform_utils.h"

int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::string filenameCfg, filenameGt, directoryScans;
    Polygon p1, p2;
    std::vector<Polygon> outputUnion;
    std::vector<Polygon> outputIntersection;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("gt", filenameGt, std::string(""));
    params.getParam<std::string>("dir", directoryScans, std::string("."));

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    // Reads the ground truth file
    std::vector<float> timesGt;
    misc_tools::VectorTransform3 transformsGt;
    misc_tools::readTimePosQuatFile(filenameGt, timesGt, transformsGt);
    std::cout << "Read " << transformsGt.size() << " ground truth poses from \""
              << filenameGt << "\"" << std::endl;

    // Open directory and read scans (not implemented here)

    // boost::geometry::intersection(p1, p2, outputIntersection);
    // boost::geometry::union_(p1, p2, outputUnion);

    return 0;
}
