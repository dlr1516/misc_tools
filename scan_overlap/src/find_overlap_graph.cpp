#include <iostream>
#include <vector>

#include <rofl/common/param_map.h>

#include "scan_overlap.h"
#include "transform_utils.h"

#define GRAPH_SIZE 10

//Quaternion: (.0, .0, 0.009603885349331175, 0.9999538816296465)
scan_overlap::Transform2 scanFrontToBase = 
    Eigen::Translation2d(0.30455923000105006, -0.005976866498877387) *
    Eigen::Rotation2Dd(2.*std::atan2(0.009603885349331175, 0.9999538816296465));
//Quaternion: (.0, .0, 0.9999853067886563, -0.005420904610579566)
scan_overlap::Transform2 scanRearToBase = 
    Eigen::Translation2d(-0.30272182866463804, -0.001624570483175795) *
    Eigen::Rotation2Dd(2.*std::atan2(0.9999853067886563, -0.005420904610579566));

scan_overlap::Transform2 scanRearToScanFront = scanFrontToBase.inverse() * scanRearToBase;

void saveForPlot(scan_overlap::Cloud cloud1, scan_overlap::Cloud cloud2, scan_overlap::Cloud cloud3);

void saveGraph( const scan_overlap::Graph& graph, 
                const std::vector<scan_overlap::Cloud>& clouds, 
                const scan_overlap::VectorTransform2& gts,
                const scan_overlap::VectorTransform2& odoms,
                const std::string& fileName);

int main(int argc, char** argv) {
    rofl::ParamMap params;
    std::string filenameCfg, filenameGt, filenameOdom,
        filenameScansFront, filenameScansRear, filenameLaser;
    int startIdx;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("gt", filenameGt, std::string(""));
    params.getParam<std::string>("scansFront", filenameScansFront, std::string(""));
    params.getParam<std::string>("scansRear", filenameScansRear, std::string(""));
    params.getParam<std::string>("laser", filenameLaser, std::string(""));
    params.getParam<std::string>("odometry", filenameOdom, std::string(""));
    params.getParam<int>("startIdx", startIdx, 100);

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    std::vector<double> timesGt, timesScansRear,
                        timesScansFront, timesOdom;
    scan_overlap::LaserSpecs laserSpecs;
    scan_overlap::VectorTransform3 transformsGt, transformsOdom;
    scan_overlap::VectorTransform2 gtGraph, odomGraph;
    std::vector<scan_overlap::Cloud> clouds, transClouds,
                                   cloudsFront, cloudsRear;
    std::string base_dir = "./poly_img";
    scan_overlap::Graph graph;
    double scanDelta;
    
    // Open file and read laser specifics
    if(!scan_overlap::readLaserSpecsFile(filenameLaser, laserSpecs)){
        std::cout << "Couldn't read laser parameters from \""
                  << filenameLaser << "\"" << std::endl;
        return -1;
    }
    std::cout << "Laser specs: " << std::endl;
    for(auto & [k, v] : laserSpecs) 
        std::cout << k << ": " << v << std::endl;
    std::cout << std::endl;

    // Reads the ground truth file
    if(!scan_overlap::readTimePosQuatFile(filenameGt, timesGt, transformsGt)){
        std::cout << "Couldn't read ground truth poses from \""
                  << filenameGt << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << transformsGt.size() << " ground truth poses from \""
              << filenameGt << "\"" << std::endl;

    // Reads odom file
    if(!scan_overlap::readTimePosQuatCovFile(filenameOdom, timesOdom, transformsOdom)){
        std::cout << "Couldn't read odometry from \""
                  << filenameOdom << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << transformsGt.size() << " odometry poses from \""
              << filenameOdom << "\"" << std::endl;

    {
        std::vector<scan_overlap::Scan> rangesFront, rangesRear;
        // Open file and read scans
        /*if(!scan_overlap::readTimeRangesFile(filenameScansRear, timesScansRear, rangesRear)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansRear << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesRear.size() << " ranges from \""
                << filenameScansRear << "\"" << std::endl;

        for(auto& scan : rangesRear){
            scan_overlap::Cloud tmp;
            scan_overlap::scanToCloud(scan, laserSpecs, tmp);
            for(auto& p : tmp) p = scanRearToScanFront*p;
            cloudsRear.push_back(tmp);
        }*/

        if(!scan_overlap::readTimeRangesFile(filenameScansFront, timesScansFront, rangesFront)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansFront << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesFront.size() << " ranges from \""
                << filenameScansFront << "\"" << std::endl;

        for(auto& scan : rangesFront){
            scan_overlap::Cloud tmp;
            scan_overlap::scanToCloud(scan, laserSpecs, tmp);
            for(auto& p : tmp) p = scanFrontToBase*p;
            cloudsFront.push_back(tmp);
        }

        /*int end = std::min(cloudsFront.size(), cloudsRear.size());
        for(int i = 0; i < end; i++) {
            scan_overlap::Cloud cloudFront, cloudRear, cloud;
            cloudFront = cloudsFront[i];
            cloudRear = cloudsRear[i];
            double sA = atan2(cloudFront.back().y(), cloudFront.back().x());
            double eA = atan2(cloudFront.front().y(), cloudFront.front().x());
            scan_overlap::fillCloud(cloudFront, cloudRear, cloud, sA, eA);//NON GARANTISCE POLIGONO!!!!!
            for(auto& p : cloud) p = scanFrontToBase*p;
            clouds.push_back(cloud);
        }*/
       clouds = cloudsFront;
    }

    int lastIdx = startIdx;
    graph.push_back(scan_overlap::Node(lastIdx));
    {
        scan_overlap::Cloud cloud = clouds[lastIdx];
        double tsF = timesScansFront[lastIdx];
        //double tsR = timesScansRear[lastIdx];
        scan_overlap::Transform3 gt3DF, gt3DR, odom3D;
        scan_overlap::Transform2 gt2DF, gt2DR, odom2D;

        if(!scan_overlap::findGtTransform(tsF, timesGt, transformsGt, gt3DF)){
            std::cout << "Couldn't find gt transform!" << std::endl;
            return -1;
        }
        scan_overlap::trans3DToTrans2D(gt3DF, gt2DF);

        if(!scan_overlap::findGtTransform(tsF, timesOdom, transformsOdom, odom3D)){
            std::cout << "Couldn't find odom transform!" << std::endl;
            return -1;
        }
        scan_overlap::trans3DToTrans2D(odom3D, odom2D);

        /*if(!scan_overlap::findGtTransform(tsR, timesGt, transformsGt, gt3DR)){
            std::cout << "Couldn't find gt transform!" << std::endl;
            return -1;
        }
        scan_overlap::trans3DToTrans2D(gt3DR, gt2DR);*/

        /*int lastF = cloudsFront[lastIdx].size();
        for(int i = 0; i < cloud.size(); i++){
            if(i < lastF)   cloud[i] = gt2DF*cloud[i];
            else            cloud[i] = gt2DR*cloud[i];
        }*/
        for(auto& p : cloud) p = gt2DF*p;
        transClouds.push_back(cloud);
        odomGraph.push_back(odom2D);
        gtGraph.push_back(gt2DF);
    }

    bool exit = false;
    while(graph.size() < GRAPH_SIZE && !exit){
        exit = true;
        for(int i = lastIdx+1; i<clouds.size(); i++){
            scan_overlap::Cloud cloud = clouds[i];
            double tsF = timesScansFront[i];
            //double tsR = timesScansRear[i];
            scan_overlap::Transform3 gt3DF, gt3DR, odom3D;
            scan_overlap::Transform2 gt2DF, gt2DR, odom2D;

            if(!scan_overlap::findGtTransform(tsF, timesGt, transformsGt, gt3DF)){
                std::cout << "Couldn't find gt transform!" << std::endl;
                break;
            }
            scan_overlap::trans3DToTrans2D(gt3DF, gt2DF);

            if(!scan_overlap::findGtTransform(tsF, timesOdom, transformsOdom, odom3D)){
            std::cout << "Couldn't find odom transform!" << std::endl;
            return -1;
            }
            scan_overlap::trans3DToTrans2D(odom3D, odom2D);

            /*if(!scan_overlap::findGtTransform(tsR, timesGt, transformsGt, gt3DR)){
                std::cout << "Couldn't find gt transform!" << std::endl;
                break;
            }
            scan_overlap::trans3DToTrans2D(gt3DR, gt2DR);*/

            //points from rear are appended at the end of front
            /*int lastF = cloudsFront[i].size();
            for(int i = 0; i < cloud.size(); i++){
                if(i < lastF)   cloud[i] = gt2DF*cloud[i];
                else            cloud[i] = gt2DR*cloud[i];
            }*/
            for(auto& p : cloud) p = gt2DF*p;

            double overlap = scan_overlap::scan_overlap(transClouds.back(), cloud);

            std::cout << overlap << std::endl;
            if(overlap < .7 && overlap > .1){
                scan_overlap::Node n(i);
                std::string dir = base_dir + "/" + 
                    std::to_string(lastIdx) + "_" + std::to_string(i);
                //scan_overlap::scan_overlap_visualization(transClouds.back(), cloud, dir);

                for(int j = 0; j < transClouds.size()-1; j++){
                    auto tCloud = transClouds[j];
                    if(scan_overlap::scan_overlap(tCloud, cloud) > .4)
                        n.adj.push_back(graph[j].id);
                }
                n.adj.push_back(graph.back().id); //adding previous node as adj, guarantees connected graph

                graph.push_back(n);
                transClouds.push_back(cloud);
                odomGraph.push_back(odom2D);
                gtGraph.push_back(gt2DF);
                lastIdx=i;
                exit=false;
                break;
            }
        }
    }
    std::cout << "Graph size: " << graph.size() << std::endl;

    //saveForPlot(cloudsFront[298], cloudsRear[298], clouds[298]);

    for(int i = 0; i < graph.size(); i++){
        auto n = graph[i];
        std::cout << "id: " << n.id << ", adj: ";
        for(auto& e : n.adj) std::cout << e << ", ";
        std::cout << std::endl;
    }

    //scan_overlap::draw_graph(graph, transClouds);

    scan_overlap::saveGraph(graph, cloudsFront, gtGraph, odomGraph, "graph_" + std::to_string(startIdx) + ".txt");

    return 0;
}

void saveForPlot(scan_overlap::Cloud cloud1, scan_overlap::Cloud cloud2, scan_overlap::Cloud cloud3){
    std::ofstream fileCloud1("front.txt");
    for(auto& p: cloud1)
        fileCloud1 << p.x() << " " << p.y() << "\n";
    auto p = cloud1.front();
    fileCloud1 << p.x() << " " << p.y() << "\n";
    fileCloud1.close(); 

    std::ofstream fileCloud2("rear.txt");
    for(auto& p: cloud2)
        fileCloud2 << p.x() << " " << p.y() << "\n";
    p = cloud2.front();
    fileCloud2 << p.x() << " " << p.y() << "\n";
    fileCloud2.close(); 

    std::ofstream fileCloud3("joined.txt");
    for(auto& p: cloud3)
        fileCloud3 << p.x() << " " << p.y() << "\n";
    p = cloud3.front();
    fileCloud3 << p.x() << " " << p.y() << "\n";
    fileCloud3.close(); 
}

