#include <iostream>
#include <Eigen/Dense>
#include <fstream>

#include "thirdparty/libicp/icpPointToPlane.h"

#include "thirdparty/libvfc/vfc.h"
// #include "../src/thirdparty/libvfc/featureMatch.h"

#include <opencv2/core/core.hpp>
#include <rofl/common/param_map.h>

using Vector2 = Eigen::Vector2d;
using Transform2 = Eigen::Affine2d;
using VectorTransform2 =
    std::vector<Transform2, Eigen::aligned_allocator<Transform2> >;
using Cloud = std::vector<Vector2>;

struct Edge{
    int src;
    int dst;

    Edge(int src_, int dst_): src(src_), dst(dst_) {}
};

struct GraphNode{
    Cloud cloud;
    int id;

    GraphNode(int id_, Cloud cloud_): id(id_), cloud(cloud_) {}
};

int readGraph(const std::string& filename, 
               std::vector<GraphNode>& nodes,
               VectorTransform2& gts,
               VectorTransform2& odoms,
               std::vector<Edge>& edges);

int main (int argc, char** argv) {
    //reading params and graph
    std::string filenameCfg, filenameGraph;
    rofl::ParamMap params;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("graph", filenameGraph, std::string(""));

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    std::vector<GraphNode> nodes;
    VectorTransform2 gts, odoms;
    std::vector<Edge> edges;

    readGraph(filenameGraph, nodes, gts, odoms, edges);

    /** 
     * ICP
     */

    // define a 3 dim problem with 10000 model points
    // and 10000 template points:
    int32_t dim = 3;
    int32_t num = 10000;

    // allocate model and template memory
    double* M = (double*) calloc(3 * num, sizeof (double));
    double* T = (double*) calloc(3 * num, sizeof (double));

    // set model and template points
    std::cout << endl << "Creating model with 10000 points ..." << endl;
    cout << "Creating template by shifting model by (1,1,1) ..." << endl;
    int32_t k = 0;
    for (double x = -2; x < 2; x += 0.04) {
        for (double y = -2; y < 2; y += 0.04) {
            double z = 5 * x * exp(-x * x - y * y);
            M[k * 3 + 0] = x;
            M[k * 3 + 1] = y;
            M[k * 3 + 2] = z;
            T[k * 3 + 0] = x - 1;
            T[k * 3 + 1] = y - 1;
            T[k * 3 + 2] = z - 1;
            k++;
        }
    }

    // start with identity as initial transformation
    // in practice you might want to use some kind of prediction here
    Matrix R = Matrix::eye(3);
    Matrix t(3, 1);

    // run point-to-plane ICP (-1 = no outlier threshold)
    cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
    IcpPointToPlane icp(M, num, dim);
    icp.fit(T, num, R, t, -1);

    // results
    cout << endl << "Transformation results:" << endl;
    cout << "R:" << endl << R << endl << endl;
    cout << "t:" << endl << t << endl << endl;

    // free memory
    free(M);
    free(T);

    /** 
     * VFC
     */

    cv::Mat img_1, img_2;
    int vfcMethod = 1; // 0: SURF; 1: ORB

     if (!img_1.data || !img_2.data
            || img_1.rows != img_2.rows || img_1.cols != img_2.cols) {
        return -1;
    }

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    std::vector< DMatch > matches, correctMatches;

    // initial matching by SURF or ORB
    // if (vfcMethod == 0) {
    //     surfInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
    //             descriptors_1, descriptors_2, matches);
    // } else {
    //     orbInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
    //             descriptors_1, descriptors_2, matches);
    // }

    // //  Remove mismatches by vector field consensus (VFC)
    // vfcMatch(keypoints_1, keypoints_2, matches, correctMatches);

    // // visualization
    // visualizeMatchingResults(img_1, img_2, keypoints_1, keypoints_2,
    //         matches, correctMatches);



    return 0;
}

int readGraph(const std::string& filename, 
               std::vector<GraphNode>& nodes,
               VectorTransform2& gts,
               VectorTransform2& odoms,
               std::vector<Edge>& edges){
    std::string line, comment, label;
	Vector2 p;
	size_t pos;
	int count;

	std::ifstream file(filename);
	if (!file) {
		std::cerr << "Cannot open file \"" << filename << "\"" << std::endl;
		return 0;
	}

	nodes.clear();
    gts.clear();
    odoms.clear();
    edges.clear();

	count = 0;
	while (!file.eof()) {
		std::getline(file, line);
		// Remove comments starting with '#'
		comment = "";
		pos = line.find_first_of('#');
		if (pos != std::string::npos) {
            if(pos == 0) continue;
			comment = line.substr(pos + 1, line.size());
			line = line.substr(0, pos);
		}
		// Parse the line (after comment removal)
        std::stringstream ssl(line);
        if(line.find("EDGE") == 0){
            int src, dst;
            ssl >> label;
            if(ssl >> src >> dst)
                edges.push_back(Edge(src, dst));
        }
        else if(line.find("NODE") == 0){
            double pX, pY, pT, oX, oY, oT;
            int id;
            Vector2 p;
            Cloud cloud;
            ssl >> label;
            if(ssl >> id >> pX >> pY >> pT >> oX >> oY >> oT){
                Transform2 odom, gt;
                odom.linear() = Eigen::Rotation2Dd(oT).matrix();
                odom.translation() = Vector2(oX, oY);

                gt.linear() = Eigen::Rotation2Dd(pT).matrix();
                gt.translation() = Vector2(pX, pY);

                odoms.push_back(odom);
                gts.push_back(gt);
            }
            else continue;

            while(ssl >> p.x() >> p.y())
                cloud.push_back(p);
            nodes.push_back(GraphNode(id, cloud));
        }
	}
	file.close();

	return count;
}