#include <iostream>
#include <vector>

#include <rofl/common/param_map.h>

#include "scan_overlap.h"
#include "transform_utils.h"


//Quaternion: (.0, .0, 0.009603885349331175, 0.9999538816296465)
misc_tools::Transform2 scanFrontToBase = 
    Eigen::Rotation2Df(2.*std::atan2(0.009603885349331175, 0.9999538816296465)) *
    Eigen::Translation2f(0.30455923000105006, -0.005976866498877387);
//Quaternion: (.0, .0, 0.9999853067886563, -0.005420904610579566)
misc_tools::Transform2 scanRearToBase = 
    Eigen::Rotation2Df(2.*std::atan2(0.9999853067886563, -0.005420904610579566)) *
    Eigen::Translation2f(-0.30272182866463804, -0.001624570483175795);


int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::string filenameCfg, filenameGt, filenameScans, filenameLaser;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("gt", filenameGt, std::string(""));
    params.getParam<std::string>("scans", filenameScans, std::string(""));
    params.getParam<std::string>("laser", filenameLaser, std::string(""));

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    // Reads the ground truth file
    std::vector<float> timesGt, timesScans;
    std::vector<misc_tools::Scan> ranges;
    misc_tools::LaserSpecs laserSpecs;
    misc_tools::VectorTransform3 transformsGt;
    if(!misc_tools::readTimePosQuatFile(filenameGt, timesGt, transformsGt)){
        std::cout << "Couldn't read ground truth poses from \""
                  << filenameGt << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << transformsGt.size() << " ground truth poses from \""
              << filenameGt << "\"" << std::endl;

    // Open file and read scans
    if(!misc_tools::readTimeRangesFile(filenameScans, timesScans, ranges)){
        std::cout << "Couldn't read ranges from \""
                  << filenameScans << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << ranges.size() << " ranges from \""
              << filenameScans << "\"" << std::endl;
    
    // Open file and read laser specifics
    if(!misc_tools::readLaserSpecsFile(filenameLaser, laserSpecs)){
        std::cout << "Couldn't read laser parameters from \""
                  << filenameLaser << "\"" << std::endl;
        return -1;
    }
    std::cout << "Laser specs: " << std::endl;
    for(auto & [k, v] : laserSpecs) 
        std::cout << k << ": " << v << std::endl;
    std::cout << std::endl;

    misc_tools::Cloud cloud1, cloud2;
    misc_tools::Scan scan;
    scan = ranges[0];
    misc_tools::scanToCloud(scan, laserSpecs, cloud1);
    std::cout << std::endl << std::endl;
    std::cout << "cloud 1 size: " << cloud1.size() << 
        ", scan 1 size: " << scan.size() << std::endl;

    for(auto&p : cloud1) p = scanFrontToBase*p;

    scan = ranges[1];
    misc_tools::scanToCloud(scan, laserSpecs, cloud2);
    std::cout << "cloud 2 size: " << cloud2.size() << 
        ", scan 2 size: " << scan.size() << std::endl;
    for(auto&p : cloud2) p = scanFrontToBase*p;

    std::cout << "First overlap: " << misc_tools::scan_overlap(cloud1, cloud2) << std::endl;

    scan = ranges[10];
    cloud2.clear();
    misc_tools::scanToCloud(scan, laserSpecs, cloud2);
    std::cout << "cloud 3 size: " << cloud2.size() << 
        ", scan 3 size: " << scan.size() << std::endl;
    for(auto&p : cloud2) p = scanFrontToBase*p;

    std::cout << "Second overlap: " << misc_tools::scan_overlap(cloud1, cloud2) << std::endl;

    return 0;
}
