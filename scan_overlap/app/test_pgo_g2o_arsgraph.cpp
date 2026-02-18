#include <iostream>
#include <cmath>

#include <rofl/common/param_map.h>

#include "simple_icp_registration.h"

#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/edge_se2_pointxy.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "types.h"
#include "transform_utils.h"

// Types for block and linear solver
typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

// class GraphSolverTransf
// {
// public:
// 	GraphSolverTransf() = default;

// 	virtual ~GraphSolverTransf() = default;

// 	virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
// 						  const scan_overlap::VectorTransform2 &odoms,
// 						  const std::vector<scan_overlap::Edge> &edges,
// 						  std::vector<scan_overlap::Transform2> &angles) = 0;
// };

// class GraphSolverTransfIcp : public GraphSolverTransf
// {
// public:
// 	GraphSolverTransfIcp() = default;

// 	virtual ~GraphSolverTransfIcp() = default;

// 	virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
// 						  const scan_overlap::VectorTransform2 &odoms,
// 						  const std::vector<scan_overlap::Edge> &edges,
// 						  std::vector<scan_overlap::Transform2> &angles) override;
// };

int main(int argc, char **argv)
{
	// mpr::MatDataReader reader;
	rofl::ParamMap params;
	std::vector<int> poseNodeIds;
	std::vector<int> landmarkNodeIds;
	std::vector<int> landmarkLabels;
	int landmarkNodeId;
	int nodeNum;

	// g2o types for vertices (VertexSE2 for poses, VertexPointXY for landmarks), edges (EdgeSE2 and EdgeSE2PointXY) and pose (SE2)
	g2o::VertexSE2 *poseVertexCurr;
	g2o::VertexSE2 *poseVertexPrev;
	g2o::VertexSE2 *transfVertex;
	g2o::EdgeSE2 *transfEdge;
	g2o::SE2 poseSE2Curr;

	// Parameters
	scan_overlap::Matrix3 odometryCovar;
	// scan_overlap::Matrix3 odometryInfo;
	scan_overlap::Matrix2 sensorCovarPolar;
	scan_overlap::Matrix2 sensorCovarCartesian, sensorJac;
	scan_overlap::Matrix2 sensorInfoCartesian;
	scan_overlap::VectorVector2 landmarks;
	std::string filenameCfg;
	// std::string filenameData;
	// std::string filenameWorld;
	double sigmaOdomPos, sigmaOdomAng, sigmaSensorRange, sigmaSensorBearing;
	int iterNum;

	bool enableIcp;

	std::string filenameGraph;

	// Reads file params from command line
	params.read(argc, argv);
	params.getParam<std::string>("cfg", filenameCfg, std::string(""));

	// Reads potential parameter value from file and, then again, from command line
	if (!params.read(filenameCfg))
	{
		std::cout << "Cannot open configuration file \"" << filenameCfg << "\": using default values" << std::endl;
	}
	params.read(argc, argv);

	params.getParam<std::string>("graph", filenameGraph, std::string(""));
	// params.getParam<std::string>("world", filenameWorld, std::string(""));
	params.getParam<double>("sigmaOdomPos", sigmaOdomPos, double(0.31623));
	params.getParam<double>("sigmaOdomAng", sigmaOdomAng, double(0.1));
	params.getParam<double>("sigmaSensorRange", sigmaSensorRange, double(0.44721));
	params.getParam<double>("sigmaSensorBearing", sigmaSensorBearing, double(0.35));
	params.getParam<int>("iterNum", iterNum, int(10));
	params.getParam<bool>("enable_icp", enableIcp, true);

	params.adaptTildeInPaths();
	params.getParam<std::string>("graph", filenameGraph, std::string(""));

	std::cout << "\nParams:\n";
	params.write(std::cout);
	std::cout << std::endl;

	// Reading data and world files
	// std::cout << "Reading file \"" << filenameData << "\"" << std::endl;
	// reader.read(filenameData);

	// mpr::readWorldFile(filenameWorld, landmarks);
	// std::cout << "read " << landmarks.size() << " landmarks from world file \"" << filenameWorld << "\"" << std::endl;

	std::vector<scan_overlap::Node> nodes;
	scan_overlap::VectorTransform2 gts, odoms;
	std::vector<scan_overlap::Edge> edges;
	std::vector<std::string> names;
	// std::vector<std::vector<double>> solutions;

	scan_overlap::readGraph(filenameGraph, nodes, gts, odoms, edges);

	ROFL_VAR4(nodes.size(), gts.size(), odoms.size(), edges.size());

	// Sets the information matrices
	// odometryCovar = scan_overlap::Matrix3::Zero();
	// odometryCovar(0, 0) = sigmaOdomPos * sigmaOdomPos;
	// odometryCovar(1, 1) = sigmaOdomPos * sigmaOdomPos;
	// odometryCovar(2, 2) = sigmaOdomAng * sigmaOdomAng;
	// odometryInfo = odometryCovar.inverse();
	// odometryInfo = 0.5 * (odometryInfo + odometryInfo.transpose());
	// std::cout << "Odometry covariance:\n"
	// 		  << odometryCovar << "\n  information matrix\n"
	// 		  << odometryInfo << std::endl;

	// sensorCovarPolar = scan_overlap::Matrix2::Zero();
	// sensorCovarPolar(0, 0) = sigmaSensorRange * sigmaSensorRange;
	// sensorCovarPolar(1, 1) = sigmaSensorBearing * sigmaSensorBearing;
	// sensorInfoCartesian = scan_overlap::Matrix2::Zero();
	// sensorInfoCartesian(0, 0) = 1.0 / (1e-6 + sigmaSensorRange * sigmaSensorRange);
	// sensorInfoCartesian(1, 1) = 1.0 / (1e-6 + sigmaSensorRange * sigmaSensorRange);
	// std::cout << "Sensor polar covariance:\n"
	// 		  << sensorCovarPolar << std::endl;

	// g2o classes:
	// 1) the optimization algorithm: g2o::SparseOptimizer;
	// 2) an optimization algorithm with Hessian like OptimizationAlgorithmGaussNewton, OptimizationAlgorithmLevenberg, OptimizationAlgorithmDogleg;
	// 3) a linear block solver (required by the optimization algorithm with Hessian): LinearSolverCholmod<>, LinearSolverCSparse<>,
	// Creates the optimizer, the main class of g2o.
	// The optimizer requires:
	// - an optimization algorithm like Gauss-Seidel;
	// - a linear solver to solve the linearized system;
	//   ---> the linearized solver operates on sparse block and needs
	//        a block solver
	g2o::SparseOptimizer optimizer;
	auto linearSolver = std::make_unique<SlamLinearSolver>();
	linearSolver->setBlockOrdering(false);
	g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(
		std::make_unique<SlamBlockSolver>(std::move(linearSolver)));

	optimizer.setAlgorithm(solver);

	// Creates the origin node
	// nodeNum = 0;
	// g2o::SE2 origin(0.0, 0.0, 0.0);
	// poseVertexCurr = new g2o::VertexSE2;
	// poseVertexCurr->setId(nodeNum);
	// poseVertexCurr->setEstimate(origin);
	// optimizer.addVertex(poseVertexCurr);
	// poseNodeIds.push_back(nodeNum);
	// nodeNum++;

	// Inserts the odometry and sensor measurements
	int numNodes = nodes.size();
	std::map<int, int> idToArsId;

	std::vector<g2o::VertexSE2 *> transfVertices;
	scan_overlap::Transform2 odomOriginInverse = odoms[0].inverse();
	for (int t = 0; t < numNodes; ++t)
	{
		// std::cout << "time step " << t << ":" << std::endl;
		idToArsId[nodes[t].id] = t;

		// Inserts the new robot pose as graph vertex
		transfVertex = new g2o::VertexSE2;
		transfVertex->setId(nodes[t].id);
		auto odom = odomOriginInverse * odoms[t];
		double x = odom.translation().x();
		double y = odom.translation().y();
		double theta = atan2(odom.linear().col(0).y(), odom.linear().col(0).x());
		g2o::SE2 nodePose = g2o::SE2(x, y, theta);
		std::cout << t << ": node id " << nodes[t].id << ", odom pose " << nodePose.translation().transpose() << ", " << RAD2DEG(nodePose.rotation().angle()) << std::endl;
		transfVertex->setEstimate(nodePose);
		if (t == 0) {
			transfVertex->setFixed(true);
		}
		transfVertices.push_back(transfVertex);
		optimizer.addVertex(transfVertex);
	}

	int numEdges = edges.size();
	std::vector<scan_overlap::Transform2> transfVecIcp;
	// if (enableIcp)
	// {
	// 	std::cout << "Starting icp..." << std::endl;
	// 	GraphSolverTransfIcp solverIcp;
	// 	solverIcp.estimate(nodes, odoms, edges, transfVecIcp);
	// 	// solutions.push_back(transfVecIcp);
	// 	names.push_back("icp");
	// }
	for (int t = 0; t < numEdges; ++t)
	{
		// Inserts the odometry edge corresponding to odometry
		transfEdge = new g2o::EdgeSE2;
		int src = idToArsId[edges[t].src];
		int dst = idToArsId[edges[t].dst];
		transfEdge->vertices()[0] = transfVertices[src];
		transfEdge->vertices()[1] = transfVertices[dst];
		auto cloudSrc = nodes[src].cloud;
		auto cloudDst = nodes[dst].cloud;
		scan_overlap::SimpleIcpRegistration icp(1000);
		scan_overlap::Transform2 transfEstim;
		icp.setPointSetSrc(cloudSrc);
		icp.setPointSetDst(cloudDst);
		auto transfGuess = scan_overlap::Transform2::Identity();
		icp.computeRigidTransform(transfEstim, transfGuess);
		g2o::SE2 edgeMeasurement(transfEstim.translation().x(), transfEstim.translation().y(),
								 atan2(transfEstim.linear().col(0).y(), transfEstim.linear().col(0).x()));
		transfEdge->setMeasurement(edgeMeasurement);
		transfEdge->setInformation(scan_overlap::Matrix3::Identity());
		transfEdge->setId(t);
		optimizer.addEdge(transfEdge);
	}

	std::cout << "graph with " << optimizer.vertices().size() << " vertices and "
			  << optimizer.edges().size() << " edges\n"
			  << std::endl;

	// std::cout << "\nInitial landmarks:\n";
	// for (int i = 0; i < landmarkNodeIds.size() && i < landmarkLabels.size(); ++i)
	// {
	// 	landmarkVertex = (g2o::VertexPointXY *)optimizer.vertex(landmarkNodeIds[i]);
	// 	std::cout << "  label " << landmarkLabels[i] << ": " << landmarkVertex->estimate().transpose() << "\n";
	// }

	optimizer.verifyInformationMatrices(true);

	std::cout << "Save before optimization" << std::endl;
	optimizer.save("before_optimization.g2o");

	optimizer.initializeOptimization();
	optimizer.setVerbose(true);
	auto activeVertices = optimizer.activeVertices();
	std::cout << "Active vertices: " << activeVertices.size() << std::endl;
	for (const auto &v : activeVertices)
	{
		std::cout << "  vertex id " << v->id() << std::endl;
	}
	auto activeEdges = optimizer.activeEdges();
	std::cout << "Active edges: " << activeEdges.size() << std::endl;
	for (const auto &e : activeEdges)
	{
		std::cout << "  edge id " << e->id() << std::endl;
	}
	optimizer.optimize(iterNum);

	std::cout << "Save after optimization" << std::endl;
	optimizer.save("after_optimization.g2o");

	// std::cout << "\nGroundtruth landmarks:\n";
	// for (int l = 0; l < landmarks.size(); ++l)
	// {
	//   std::cout << "  label " << (l + 1) << ": " << landmarks[l].transpose() << "\n";
	// }
	// std::cout << std::endl;

	// std::cout << "\nEstimated landmarks:\n";
	// for (int i = 0; i < landmarkNodeIds.size() && i < landmarkLabels.size(); ++i)
	// {
	//   landmarkVertex = (g2o::VertexPointXY *)optimizer.vertex(landmarkNodeIds[i]);
	//   std::cout << "  label " << landmarkLabels[i] << ": " << landmarkVertex->estimate().transpose() << "\n";
	// }

	for (int t = 0; t < numNodes; ++t)
	{
		auto vertex = (g2o::VertexSE2 *)optimizer.vertex(nodes[t].id);
		auto estPose = vertex->estimate();
		std::cout << "Node " << nodes[t].id << ": estimated pose " << estPose.translation().transpose() << ", " << RAD2DEG(estPose.rotation().angle()) << std::endl;
	}

	optimizer.clear();

	return 0;
}

// void GraphSolverTransfIcp::estimate(const std::vector<scan_overlap::Node> &nodes,
// 									const scan_overlap::VectorTransform2 &odoms,
// 									const std::vector<scan_overlap::Edge> &edges,
// 									std::vector<scan_overlap::Transform2> &transfVec)
// {
// 	scan_overlap::SimpleIcpRegistration icp(1000);
// 	scan_overlap::Transform2 transfEstim;
// 	scan_overlap::Transform2 transfGuess;
// 	scan_overlap::Transform2 transfGlobal;

// 	transfVec.clear();
// 	transfGlobal.setIdentity();
// 	transfVec.push_back(scan_overlap::Transform2::Identity());
// 	for (int i = 1; i < nodes.size(); ++i)
// 	{
// 		const auto &nodeSrc = nodes.at(i - 1);
// 		const auto &nodeDst = nodes.at(i);
// 		transfGuess = odoms.at(i - 1).inverse() * odoms.at(i);
// 		icp.setPointSetSrc(nodeSrc.cloud);
// 		icp.setPointSetDst(nodeDst.cloud);
// 		icp.computeRigidTransform(transfEstim, transfGuess);
// 		transfGlobal = transfGlobal * transfEstim;
// 		auto tgl = transfGlobal.linear().col(0);
// 		// transfVec.push_back(atan2(tgl.y(), tgl.x()));
// 		transfVec.push_back(transfGlobal);
// 	}
// }