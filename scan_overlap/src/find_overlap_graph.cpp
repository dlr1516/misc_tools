#include <iostream>
#include <vector>

#include <rofl/common/param_map.h>

#include "scan_overlap.h"
#include "transform_utils.h"

#define GRAPH_SIZE 10


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
    std::string filenameCfg, filenameGt, 
        filenameScansFront, filenameScansRear, filenameLaser;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("gt", filenameGt, std::string(""));
    params.getParam<std::string>("scansFront", filenameScansFront, std::string(""));
    params.getParam<std::string>("scansRear", filenameScansRear, std::string(""));
    params.getParam<std::string>("laser", filenameLaser, std::string(""));

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    std::vector<double> timesGt, timesScans;
    misc_tools::LaserSpecs laserSpecs;
    misc_tools::VectorTransform3 transformsGt;
    std::vector<misc_tools::Cloud> clouds, transClouds;
    std::string base_dir = "./poly_img";
    misc_tools::Graph graph;
    
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
    // Reads the ground truth file
    if(!misc_tools::readTimePosQuatFile(filenameGt, timesGt, transformsGt)){
        std::cout << "Couldn't read ground truth poses from \""
                  << filenameGt << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << transformsGt.size() << " ground truth poses from \""
              << filenameGt << "\"" << std::endl;

    {
        std::vector<misc_tools::Scan> rangesFront, rangesRear;
        std::vector<misc_tools::Cloud> cloudsFront, cloudsRear;
        // Open file and read scans
        if(!misc_tools::readTimeRangesFile(filenameScansRear, timesScans, rangesRear)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansRear << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesRear.size() << " ranges from \""
                << filenameScansRear << "\"" << std::endl;

        for(auto& scan : rangesRear){
            misc_tools::Cloud tmp;
            misc_tools::scanToCloud(scan, laserSpecs, tmp);
            //for(auto& p : tmp) p = scanRearToBase*p;
            cloudsRear.push_back(tmp);
        }

        timesScans.clear();
        if(!misc_tools::readTimeRangesFile(filenameScansFront, timesScans, rangesFront)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansFront << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesFront.size() << " ranges from \""
                << filenameScansFront << "\"" << std::endl;

        for(auto& scan : rangesFront){
            misc_tools::Cloud tmp;
            misc_tools::scanToCloud(scan, laserSpecs, tmp);
            //for(auto& p : tmp) p = scanFrontToBase*p;
            cloudsFront.push_back(tmp);
        }

        int end = std::min(cloudsFront.size(), cloudsRear.size());
        for(int i = 0; i < end; i++) {
            misc_tools::Cloud cloudFront, cloudRear, cloud;
            cloudFront = cloudsFront[i];
            cloudRear = cloudsRear[i];
            misc_tools::joinClouds(cloudFront, cloudRear, cloud);
            clouds.push_back(cloud);
        }
        clouds = cloudsRear;
    }

    int lastIdx = 1;
    graph.push_back(misc_tools::Node(lastIdx));
    {
        misc_tools::Cloud cloud = clouds[lastIdx];
        double ts = timesScans[lastIdx];
        misc_tools::Transform3 gt3D;
        misc_tools::Transform2 gt2D;
        if(!misc_tools::findGtTransform(ts, timesGt, transformsGt, gt3D)){
            std::cout << "Couldn't find gt transform for first cloud!" << std::endl;
            return -1;
        }

        misc_tools::trans3DToTrans2D(gt3D, gt2D);
        //for(auto& p : cloud) p = gt2D*p;
        double oldAngle = -INFINITY;
        for(auto& p : cloud){
            double angle = atan2(p.y(), p.x());
            if (angle < oldAngle) std::cout << "___OH NO!___";
            std::cout << angle << "; ";
            oldAngle = angle;
        }
        std::cout << std::endl << std::endl;
        transClouds.push_back(cloud);
    }
    bool exit = false;
    while(graph.size() < GRAPH_SIZE && !exit){
        exit = true;
        for(int i = lastIdx+1; i<clouds.size(); i++){
            misc_tools::Cloud cloud = clouds[i];
            double ts = timesScans[i];
            misc_tools::Transform3 gt3D;
            misc_tools::Transform2 gt2D;

            if(!misc_tools::findGtTransform(ts, timesGt, transformsGt, gt3D)){
                std::cout << "Couldn't find gt transform!" << std::endl;
                break;
            }

            misc_tools::trans3DToTrans2D(gt3D, gt2D);
            //for(auto& p : cloud) p = gt2D*p;

            double overlap = misc_tools::scan_overlap(transClouds.back(), cloud);
            std::cout << overlap << std::endl;
            if(overlap < .75){
                std::string dir = base_dir + "/" + 
                    std::to_string(lastIdx) + "_" + std::to_string(i);
                misc_tools::scan_overlap_visualization(transClouds.back(), cloud, dir);

                graph.push_back(misc_tools::Node(i));
                transClouds.push_back(cloud);
                lastIdx=i;
                exit=false;
                break;
            }
        }
    }
    std::cout << "Graph size: " << graph.size() << std::endl;

    for(auto& n : graph){
        std::cout << "id: " << n.id << std::endl;
    }

    return 0;
}
