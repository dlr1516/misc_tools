#include <iostream>
#include <cmath>

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

#include <rofl/common/param_map.h>
#include "types.h"

// Types for block and linear solver
typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>> SlamBlockSolver;
typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

/**
 * Returns the landmark node IDs if in landmarkLabels[] there is the required label.
 * Otherwise returns -1.
 */
int findLandmarkNodeId(const std::vector<int> &landmarkNodeIds, const std::vector<int> &landmarkLabels, int label);

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
  g2o::VertexPointXY *landmarkVertex;
  g2o::EdgeSE2 *odometryEdge;
  g2o::EdgeSE2PointXY *landmarkEdge;
  g2o::SE2 poseSE2Curr;

  // Parameters
  scan_overlap::Matrix3 odometryCovar;
  scan_overlap::Matrix3 odometryInfo;
  scan_overlap::Matrix2 sensorCovarPolar;
  scan_overlap::Matrix2 sensorCovarCartesian, sensorJac;
  scan_overlap::Matrix2 sensorInfoCartesian;
  scan_overlap::VectorVector2 landmarks;
  std::string filenameCfg;
  std::string filenameData;
  std::string filenameWorld;
  double sigmaOdomPos, sigmaOdomAng, sigmaSensorRange, sigmaSensorBearing;
  int iterNum;

  // Reads file params from command line
  params.read(argc, argv);
  params.getParam<std::string>("cfg", filenameCfg, std::string(""));

  // Reads potential parameter value from file and, then again, from command line
  if (!params.read(filenameCfg))
  {
    std::cout << "Cannot open configuration file \"" << filenameCfg << "\": using default values" << std::endl;
  }
  params.read(argc, argv);

  params.getParam<std::string>("data", filenameData, std::string(""));
  params.getParam<std::string>("world", filenameWorld, std::string(""));
  params.getParam<double>("sigmaOdomPos", sigmaOdomPos, double(0.31623));
  params.getParam<double>("sigmaOdomAng", sigmaOdomAng, double(0.1));
  params.getParam<double>("sigmaSensorRange", sigmaSensorRange, double(0.44721));
  params.getParam<double>("sigmaSensorBearing", sigmaSensorBearing, double(0.35));
  params.getParam<int>("iterNum", iterNum, int(10));

  std::cout << "\nParams:\n";
  params.write(std::cout);
  std::cout << std::endl;

  // Reading data and world files
  std::cout << "Reading file \"" << filenameData << "\"" << std::endl;
  // reader.read(filenameData);

  // mpr::readWorldFile(filenameWorld, landmarks);
  std::cout << "read " << landmarks.size() << " landmarks from world file \"" << filenameWorld << "\"" << std::endl;

  // Sets the information matrices
  odometryCovar = scan_overlap::Matrix3::Zero();
  odometryCovar(0, 0) = sigmaOdomPos * sigmaOdomPos;
  odometryCovar(1, 1) = sigmaOdomPos * sigmaOdomPos;
  odometryCovar(2, 2) = sigmaOdomAng * sigmaOdomAng;
  odometryInfo = odometryCovar.inverse();
  odometryInfo = 0.5 * (odometryInfo + odometryInfo.transpose());
  std::cout << "Odometry covariance:\n"
            << odometryCovar << "\n  information matrix\n"
            << odometryInfo << std::endl;

  sensorCovarPolar = Eigen::Matrix2d::Zero();
  sensorCovarPolar(0, 0) = sigmaSensorRange * sigmaSensorRange;
  sensorCovarPolar(1, 1) = sigmaSensorBearing * sigmaSensorBearing;
  sensorInfoCartesian = Eigen::Matrix2d::Zero();
  sensorInfoCartesian(0, 0) = 1.0 / (1e-6 + sigmaSensorRange * sigmaSensorRange);
  sensorInfoCartesian(1, 1) = 1.0 / (1e-6 + sigmaSensorRange * sigmaSensorRange);
  std::cout << "Sensor polar covariance:\n"
            << sensorCovarPolar << std::endl;

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
  nodeNum = 0;
  g2o::SE2 origin(0.0, 0.0, 0.0);
  poseVertexCurr = new g2o::VertexSE2;
  poseVertexCurr->setId(nodeNum);
  poseVertexCurr->setEstimate(origin);
  poseVertexCurr->setFixed(true);
  optimizer.addVertex(poseVertexCurr);
  poseNodeIds.push_back(nodeNum);
  nodeNum++;

  // Inserts the odometry and sensor measurements
  int readerStepNum = 10; // reader.getStepNumber(); // TODO 1
  for (int t = 0; t < readerStepNum; ++t)
  {
    // std::cout << "time step " << t << ":" << std::endl;

    // Inserts the new robot pose as graph vertex
    poseVertexPrev = (g2o::VertexSE2 *)optimizer.vertex(poseNodeIds.back());

    g2o::SE2 readerGetDataItemTodom; // = reader.getDataItem(t).odometry; // TODO 2
    poseSE2Curr = poseVertexPrev->estimate() * readerGetDataItemTodom;
    poseVertexCurr = new g2o::VertexSE2;
    poseVertexCurr->setId(nodeNum);
    poseVertexCurr->setEstimate(poseSE2Curr);
    optimizer.addVertex(poseVertexCurr);
    poseNodeIds.push_back(nodeNum);
    nodeNum++;

    // Inserts the odometry edge corresponding to odometry
    odometryEdge = new g2o::EdgeSE2;
    odometryEdge->vertices()[0] = poseVertexPrev;
    odometryEdge->vertices()[1] = poseVertexCurr;
    odometryEdge->setMeasurement(readerGetDataItemTodom);
    odometryEdge->setInformation(odometryInfo);
    optimizer.addEdge(odometryEdge);

    // Inserts the sensor measurements
    int sensorNum; // = reader.getDataItem(t).sensors.size(); // TODO 3
    for (int i = 0; i < sensorNum; ++i)
    {
      // Inserts the landmark vertex only if it does not exists!
      // We do this using the "oracles" given by the ids
      int label; // = reader.getDataItem(t).sensors[i].label; // TODO 4
      landmarkNodeId = findLandmarkNodeId(landmarkNodeIds, landmarkLabels, label);
      if (landmarkNodeId < 0)
      {
        landmarkVertex = new g2o::VertexPointXY;
        landmarkVertex->setId(nodeNum);
        g2o::Vector2 sensorPose; // = poseSE2Curr * reader.getDataItem(t).sensors[i].position; // TODO 5a
        landmarkVertex->setEstimate(poseSE2Curr * sensorPose);
        // landmarkVertex->setMarginalized(true);
        optimizer.addVertex(landmarkVertex);
        landmarkNodeIds.push_back(nodeNum);
        landmarkLabels.push_back(label);
        nodeNum++;
      }
      else
      {
        landmarkVertex = (g2o::VertexPointXY *)optimizer.vertex(landmarkNodeId);
      }

      // Computes the sensor information matrix in cartesian coordinates
      //            double r = reader.getDataItem(t).sensors[i].range;
      //            double a = reader.getDataItem(t).sensors[i].bearing;
      //            sensorJac << cos(a), -r * sin(a),
      //                    sin(a), r * cos(a);
      //            sensorCovarCartesian = sensorJac * sensorCovarPolar * sensorJac.transpose();
      //            sensorInfoCartesian = sensorCovarCartesian.inverse();
      //            sensorInfoCartesian = 0.5 * (sensorInfoCartesian + sensorInfoCartesian.transpose());

      // Adds the measurement
      landmarkEdge = new g2o::EdgeSE2PointXY;
      landmarkEdge->vertices()[0] = poseVertexCurr;
      landmarkEdge->vertices()[1] = landmarkVertex;
      g2o::Vector2 sensorPose; // = poseSE2Curr * reader.getDataItem(t).sensors[i].position; // TODO 5b
      landmarkEdge->setMeasurement(sensorPose);
      landmarkEdge->setInformation(sensorInfoCartesian);
      optimizer.addEdge(landmarkEdge);
    }
  }

  std::cout << "graph with " << optimizer.vertices().size() << " vertices and "
            << optimizer.edges().size() << " edges\n"
            << "  poses " << poseNodeIds.size() << ", landmarks " << landmarkNodeIds.size()
            << std::endl;

  std::cout << "\nInitial landmarks:\n";
  for (int i = 0; i < landmarkNodeIds.size() && i < landmarkLabels.size(); ++i)
  {
    landmarkVertex = (g2o::VertexPointXY *)optimizer.vertex(landmarkNodeIds[i]);
    std::cout << "  label " << landmarkLabels[i] << ": " << landmarkVertex->estimate().transpose() << "\n";
  }

  optimizer.verifyInformationMatrices(true);

  std::cout << "Save before optimization" << std::endl;
  optimizer.save("before_optimization.g2o");

  optimizer.initializeOptimization();
  optimizer.optimize(iterNum);

  std::cout << "Save after optimization" << std::endl;
  optimizer.save("after_optimization.g2o");

  std::cout << "\nGroundtruth landmarks:\n";
  for (int l = 0; l < landmarks.size(); ++l)
  {
    std::cout << "  label " << (l + 1) << ": " << landmarks[l].transpose() << "\n";
  }
  std::cout << std::endl;

  std::cout << "\nEstimated landmarks:\n";
  for (int i = 0; i < landmarkNodeIds.size() && i < landmarkLabels.size(); ++i)
  {
    landmarkVertex = (g2o::VertexPointXY *)optimizer.vertex(landmarkNodeIds[i]);
    std::cout << "  label " << landmarkLabels[i] << ": " << landmarkVertex->estimate().transpose() << "\n";
  }

  optimizer.clear();

  return 0;
}

int findLandmarkNodeId(const std::vector<int> &landmarkNodeIds, const std::vector<int> &landmarkLabels, int label)
{
  if (landmarkNodeIds.size() != landmarkLabels.size())
  {
    std::cerr << "inconstent numbers of nodes " << landmarkNodeIds.size() << " and labels " << landmarkLabels.size() << std::endl;
    return -1;
  }
  int n = landmarkNodeIds.size();
  for (int i = 0; i < n; ++i)
  {
    if (landmarkLabels[i] == label)
    {
      return landmarkNodeIds[i];
    }
  }
  return -1;
}
