#include <iostream>
#include <vector>

#include <rofl/common/param_map.h>

#include "scan_overlap.h"
#include "transform_utils.h"

#define GRAPH_SIZE 10

//Quaternion: (.0, .0, 0.009603885349331175, 0.9999538816296465)
misc_tools::Transform2 scanFrontToBase = 
    Eigen::Translation2d(0.30455923000105006, -0.005976866498877387) *
    Eigen::Rotation2Dd(2.*std::atan2(0.009603885349331175, 0.9999538816296465));
//Quaternion: (.0, .0, 0.9999853067886563, -0.005420904610579566)
misc_tools::Transform2 scanRearToBase = 
    Eigen::Translation2d(-0.30272182866463804, -0.001624570483175795) *
    Eigen::Rotation2Dd(2.*std::atan2(0.9999853067886563, -0.005420904610579566));

misc_tools::Transform2 scanRearToScanFront = scanFrontToBase.inverse() * scanRearToBase;

void saveForPlot(misc_tools::Cloud cloud1, misc_tools::Cloud cloud2, misc_tools::Cloud cloud3);

void saveGraph( const misc_tools::Graph& graph, 
                const std::vector<misc_tools::Cloud>& clouds, 
                const misc_tools::VectorTransform2& gts,
                const misc_tools::VectorTransform2& odoms,
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
    misc_tools::LaserSpecs laserSpecs;
    misc_tools::VectorTransform3 transformsGt, transformsOdom;
    misc_tools::VectorTransform2 gtGraph, odomGraph;
    std::vector<misc_tools::Cloud> clouds, transClouds,
                                   cloudsFront, cloudsRear;
    std::string base_dir = "./poly_img";
    misc_tools::Graph graph;
    double scanDelta;
    
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

    // Reads odom file
    if(!misc_tools::readTimePosQuatCovFile(filenameOdom, timesOdom, transformsOdom)){
        std::cout << "Couldn't read odometry from \""
                  << filenameOdom << "\"" << std::endl;
        return -1;
    }
    std::cout << "Read " << transformsGt.size() << " odometry poses from \""
              << filenameOdom << "\"" << std::endl;

    {
        std::vector<misc_tools::Scan> rangesFront, rangesRear;
        // Open file and read scans
        /*if(!misc_tools::readTimeRangesFile(filenameScansRear, timesScansRear, rangesRear)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansRear << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesRear.size() << " ranges from \""
                << filenameScansRear << "\"" << std::endl;

        for(auto& scan : rangesRear){
            misc_tools::Cloud tmp;
            misc_tools::scanToCloud(scan, laserSpecs, tmp);
            for(auto& p : tmp) p = scanRearToScanFront*p;
            cloudsRear.push_back(tmp);
        }*/

        if(!misc_tools::readTimeRangesFile(filenameScansFront, timesScansFront, rangesFront)){
            std::cout << "Couldn't read ranges from \""
                    << filenameScansFront << "\"" << std::endl;
            return -1;
        }
        std::cout << "Read " << rangesFront.size() << " ranges from \""
                << filenameScansFront << "\"" << std::endl;

        for(auto& scan : rangesFront){
            misc_tools::Cloud tmp;
            misc_tools::scanToCloud(scan, laserSpecs, tmp);
            for(auto& p : tmp) p = scanFrontToBase*p;
            cloudsFront.push_back(tmp);
        }

        /*int end = std::min(cloudsFront.size(), cloudsRear.size());
        for(int i = 0; i < end; i++) {
            misc_tools::Cloud cloudFront, cloudRear, cloud;
            cloudFront = cloudsFront[i];
            cloudRear = cloudsRear[i];
            double sA = atan2(cloudFront.back().y(), cloudFront.back().x());
            double eA = atan2(cloudFront.front().y(), cloudFront.front().x());
            misc_tools::fillCloud(cloudFront, cloudRear, cloud, sA, eA);//NON GARANTISCE POLIGONO!!!!!
            for(auto& p : cloud) p = scanFrontToBase*p;
            clouds.push_back(cloud);
        }*/
       clouds = cloudsFront;
    }

    int lastIdx = startIdx;
    graph.push_back(misc_tools::Node(lastIdx));
    {
        misc_tools::Cloud cloud = clouds[lastIdx];
        double tsF = timesScansFront[lastIdx];
        //double tsR = timesScansRear[lastIdx];
        misc_tools::Transform3 gt3DF, gt3DR, odom3D;
        misc_tools::Transform2 gt2DF, gt2DR, odom2D;

        if(!misc_tools::findGtTransform(tsF, timesGt, transformsGt, gt3DF)){
            std::cout << "Couldn't find gt transform!" << std::endl;
            return -1;
        }
        misc_tools::trans3DToTrans2D(gt3DF, gt2DF);

        if(!misc_tools::findGtTransform(tsF, timesOdom, transformsOdom, odom3D)){
            std::cout << "Couldn't find odom transform!" << std::endl;
            return -1;
        }
        misc_tools::trans3DToTrans2D(odom3D, odom2D);

        /*if(!misc_tools::findGtTransform(tsR, timesGt, transformsGt, gt3DR)){
            std::cout << "Couldn't find gt transform!" << std::endl;
            return -1;
        }
        misc_tools::trans3DToTrans2D(gt3DR, gt2DR);*/

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
            misc_tools::Cloud cloud = clouds[i];
            double tsF = timesScansFront[i];
            //double tsR = timesScansRear[i];
            misc_tools::Transform3 gt3DF, gt3DR, odom3D;
            misc_tools::Transform2 gt2DF, gt2DR, odom2D;

            if(!misc_tools::findGtTransform(tsF, timesGt, transformsGt, gt3DF)){
                std::cout << "Couldn't find gt transform!" << std::endl;
                break;
            }
            misc_tools::trans3DToTrans2D(gt3DF, gt2DF);

            if(!misc_tools::findGtTransform(tsF, timesOdom, transformsOdom, odom3D)){
            std::cout << "Couldn't find odom transform!" << std::endl;
            return -1;
            }
            misc_tools::trans3DToTrans2D(odom3D, odom2D);

            /*if(!misc_tools::findGtTransform(tsR, timesGt, transformsGt, gt3DR)){
                std::cout << "Couldn't find gt transform!" << std::endl;
                break;
            }
            misc_tools::trans3DToTrans2D(gt3DR, gt2DR);*/

            //points from rear are appended at the end of front
            /*int lastF = cloudsFront[i].size();
            for(int i = 0; i < cloud.size(); i++){
                if(i < lastF)   cloud[i] = gt2DF*cloud[i];
                else            cloud[i] = gt2DR*cloud[i];
            }*/
            for(auto& p : cloud) p = gt2DF*p;

            double overlap = misc_tools::scan_overlap(transClouds.back(), cloud);

            std::cout << overlap << std::endl;
            if(overlap < .7 && overlap > .1){
                misc_tools::Node n(i);
                std::string dir = base_dir + "/" + 
                    std::to_string(lastIdx) + "_" + std::to_string(i);
                misc_tools::scan_overlap_visualization(transClouds.back(), cloud, dir);

                for(int j = 0; j < transClouds.size()-1; j++){
                    auto tCloud = transClouds[j];
                    if(misc_tools::scan_overlap(tCloud, cloud) > .4)
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

    misc_tools::draw_graph(graph, transClouds);

    saveGraph(graph, cloudsFront, gtGraph, odomGraph, "graph_" + std::to_string(startIdx) + ".txt");

    return 0;
}

void saveForPlot(misc_tools::Cloud cloud1, misc_tools::Cloud cloud2, misc_tools::Cloud cloud3){
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

void saveGraph( const misc_tools::Graph& graph, 
                const std::vector<misc_tools::Cloud>& clouds, 
                const misc_tools::VectorTransform2& gts,
                const misc_tools::VectorTransform2& odoms,
                const std::string& fileName){
    std::ofstream f(fileName);
    f << "# format\n" <<
         "# NODE id poseX poseY poseTheta odomX odomY odomTheta x0 y0 ... xN yN\n" <<
         "# EDGE idSrc idDst\n";
    for(int i = 0; i < graph.size(); i++){
        misc_tools::Node n = graph[i];
        misc_tools::Cloud cloud = clouds[n.id];

        misc_tools::Transform2 gt = gts[i];
        Eigen::Translation2d gtT = Eigen::Translation2d(gt.translation());
        double gtA = Eigen::Rotation2Dd(gt.linear()).angle();

        misc_tools::Transform2 odom = odoms[i];
        Eigen::Translation2d odomT = Eigen::Translation2d(odom.translation());
        double odomA = Eigen::Rotation2Dd(odom.linear()).angle();

        f << "NODE " << n.id << " " << gtT.x() << " " << gtT.y() << " " << gtA << " " <<
             odomT.x() << " " << odomT.y() << " " << odomA << " ";

        for(const auto& p : cloud) f << p.x() << " " << p.y() << " ";
        
        f << "\n";
        
        for(int a : n.adj) f << "EDGE " << a << " " << n.id << "\n";
    }
}