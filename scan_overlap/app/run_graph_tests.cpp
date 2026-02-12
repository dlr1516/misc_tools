#include <iostream>
#include <map>
#include <Eigen/Dense>
#include <fstream>

#include <rofl/common/param_map.h>

#include <ars/ars2d.h>
#include <ars/utils.h>
#include <ars/ArsGraph.h>
#include <ars/ArsGraphSolver.h>

#include <opencv2/core/core.hpp>

#include "simple_icp_registration.h"
#include "vfc_registration.h"

#include "thirdparty/libicp/icpPointToPlane.h"

#include "thirdparty/libvfc/vfc.h"
// #include "thirdparty/libvfc/featureMatch.h"

#include "transform_utils.h"

#include <rofl/common/profiler.h>
#include <rofl/common/io.h>

class GraphSolver
{
public:
    GraphSolver() = default;

    virtual ~GraphSolver() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) = 0;
};

class GraphSolverIcp : public GraphSolver
{
public:
    GraphSolverIcp() = default;

    virtual ~GraphSolverIcp() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) override;
};

class GraphSolverVfc : public GraphSolver
{
public:
    GraphSolverVfc() = default;

    virtual ~GraphSolverVfc() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) override;
};

class GraphSolverArs : public GraphSolver
{
public:
    GraphSolverArs() = default;

    virtual ~GraphSolverArs() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) override;
};

class GraphSolverArsGraph : public GraphSolver
{
public:
    GraphSolverArsGraph() = default;

    virtual ~GraphSolverArsGraph() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) override;
};

double mod180(double angle);

int main(int argc, char **argv)
{
    // reading params and graph
    std::string filenameCfg, filenameGraph;
    bool enableArs, enableIcp, enableVfc, enableArsGraph;
    rofl::ParamMap params;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("in", filenameGraph, std::string(""));
    params.getParam<bool>("enable_ars", enableArs, false);
    params.getParam<bool>("enable_icp", enableIcp, false);
    params.getParam<bool>("enable_vfc", enableVfc, false);
    params.getParam<bool>("enable_ars_graph", enableArsGraph, false);

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n"
              << std::endl;

    std::vector<scan_overlap::Node> nodes;
    scan_overlap::VectorTransform2 gts, odoms;
    std::vector<scan_overlap::Edge> edges;

    scan_overlap::readGraph(filenameGraph, nodes, gts, odoms, edges);

    ROFL_VAR4(nodes.size(), gts.size(), odoms.size(), edges.size());

    /**
     * ARS
     */
    // compute coeffs, used for both ars pairwise and ars graph

    if (enableArs || enableArsGraph)
        for (auto &n : nodes)
        {
            std::cout << "Node " << n.id << ", cloud size " << n.cloud.size() << std::endl;

            ars::AngularRadonSpectrum2d arsSrc;

            int fourierOrder = 30; // TODO: make ARS params configurable from file/command line
            double sigma = 0.05;

            arsSrc.setARSFOrder(fourierOrder);

            arsSrc.initLUT(0.0001);
            arsSrc.setComputeMode(ars::ArsKernelIsotropic2d::ComputeMode::PNEBI_LUT);

            auto timeStart = std::chrono::system_clock::now();
            ars::VectorVector2 acesPoints1;
            for (auto &p : n.cloud)
                acesPoints1.push_back(ars::Vector2(p.x(), p.y()));

            arsSrc.insertIsotropicGaussians(acesPoints1, sigma);

            std::cout << "ars.coefficients().at(0) " << arsSrc.coefficients().at(0) << ", ars.coefficients().at(2) " << arsSrc.coefficients().at(2) << std::endl;

            auto timeStop = std::chrono::system_clock::now();
            double timeAvg = (double)std::chrono::duration_cast<std::chrono::milliseconds>(timeStop - timeStart).count();
            std::cout << "insertIsotropicGaussians() " << timeAvg << " ms" << std::endl;

            std::cout << "\n------\n"
                      << std::endl;

            std::cout << "\nARS Coefficients:\n";
            std::cout << "\ti \tLUT\n";
            for (int i = 0; i < arsSrc.coefficients().size(); ++i)
            {
                std::cout << "\t" << i << " \t" << arsSrc.coefficients().at(i) << "\n";
            }
            std::cout << std::endl;

            n.setCoeffs(arsSrc.coefficients());
        }

    // scan_overlap::Node src = nodes.front();
    // scan_overlap::Node dst = nodes.back();

    // std::vector<double> funcFourierRecursDownLUT;
    // std::vector<double> funcFourierRecursDown;
    // int thnum = 360;
    // double dtheta = M_PI / thnum;
    // double theta;
    // for (int i = 0; i < thnum; ++i)
    // {
    //     theta = dtheta * i;
    //     funcFourierRecursDownLUT.push_back(ars::evaluateFourier(src.coeffs, theta));
    //     funcFourierRecursDown.push_back(ars::evaluateFourier(dst.coeffs, theta));
    // }

    // std::vector<double> correlationFourier;
    // ars::computeFourierCorr(src.coeffs, dst.coeffs, correlationFourier);

    // double arsThetaToll = 1.0f, fourierTol = 1.0f;
    // double thetaMax, corrMax;
    // ars::findGlobalMaxBBFourier(correlationFourier, 0.0, M_PI, arsThetaToll, fourierTol, thetaMax, corrMax);
    // double rotArs = thetaMax;

    /**
     * ICP
     */

    std::vector<double> anglesIcp;
    if (enableIcp)
    {
        GraphSolverIcp solverIcp;
        solverIcp.estimate(nodes, odoms, edges, anglesIcp);
    }

    /**
     * VFC
     */

    std::vector<double> anglesVfc;
    if (enableVfc)
    {
        GraphSolverVfc solverVfc;
        solverVfc.estimate(nodes, odoms, edges, anglesVfc);
    }

    /**
     * ARS
     */
    std::vector<double> anglesArs;
    if (enableArs)
    {
        GraphSolverArs solverArs;
        solverArs.estimate(nodes, odoms, edges, anglesArs);
    }

    /**
     * ARS Graph
     */
    std::vector<double> anglesArsGraph;
    if (enableArsGraph)
    {
        GraphSolverArsGraph solverArsGraph;
        solverArsGraph.estimate(nodes, odoms, edges, anglesArsGraph);
    }

    ROFL_VAR4(enableIcp, enableVfc, enableArs, enableArsGraph);
    ROFL_VAR4(anglesIcp.size(), anglesVfc.size(), anglesArs.size(), anglesArsGraph.size());

    /**
     * Gathering results
     */
    std::string methodsEnabled = "";
    if (enableIcp)
        methodsEnabled += "icp_";
    if (enableVfc)
        methodsEnabled += "vfc_";
    if (enableArs)
        methodsEnabled += "arspw_";
    if (enableArsGraph)
        methodsEnabled += "arsgraph_";
    ROFL_VAR1(methodsEnabled);
    std::string filenameOut = rofl::generateStampedString("results_" + methodsEnabled, ".csv");
    ROFL_VAR1(filenameOut);
    std::ofstream fileOut(filenameOut);
    fileOut << "id,\tgt,\t\todom,";
    if (enableIcp)
        fileOut << "\t\ticp,";
    if (enableVfc)
        fileOut << "\t\tvfc,";
    if (enableArs)
        fileOut << "\t\tars,";
    if (enableArsGraph)
        fileOut << "\t\tars_graph";
    fileOut << "\n";
    const auto& gt0inv = gts.front().inverse();
    const auto& odom0inv = odoms.front().inverse();
    for (int i = 0; i < nodes.size(); ++i)
    {
        auto gt = gt0inv * gts.at(i);
        auto odom = odom0inv*odoms.at(i);
        fileOut << std::fixed << std::setprecision(5)
                << nodes.at(i).id << ",\t" << RAD2DEG(atan2(gt.linear().col(0).y(), gt.linear().col(0).x())) 
                << ",\t" << RAD2DEG(atan2(odom.linear().col(0).y(), odom.linear().col(0).x())) << ",\t";
        if (enableIcp)
            if (i < anglesIcp.size())
                fileOut << RAD2DEG(anglesIcp.at(i)) << ",\t";
        if (enableVfc)
            if (i < anglesVfc.size())
                fileOut << RAD2DEG(anglesVfc.at(i)) << ",\t";
        if (enableArs)
            if (i < anglesArs.size())
                fileOut << RAD2DEG(anglesArs.at(i)) << ",\t";
        if (enableArsGraph)
            if (i < anglesArsGraph.size())
                fileOut << RAD2DEG(anglesArsGraph.at(i));
        fileOut << "\n";
    }

    return 0;
}

void GraphSolverIcp::estimate(const std::vector<scan_overlap::Node> &nodes,
                              const scan_overlap::VectorTransform2 &odoms,
                              const std::vector<scan_overlap::Edge> &edges,
                              std::vector<double> &angles)
{
    scan_overlap::SimpleIcpRegistration icp(1000);
    scan_overlap::Transform2 transfEstim;
    scan_overlap::Transform2 transfGuess;
    scan_overlap::Transform2 transfGlobal;

    transfGlobal.setIdentity();
    angles.push_back(.0);
    for (int i = 1; i < nodes.size(); ++i)
    {
        const auto &nodeSrc = nodes.at(i - 1);
        const auto &nodeDst = nodes.at(i);
        transfGuess = odoms.at(i - 1).inverse() * odoms.at(i);
        icp.setPointSetSrc(nodeSrc.cloud);
        icp.setPointSetDst(nodeDst.cloud);
        icp.computeRigidTransform(transfEstim, transfGuess);
        transfGlobal = transfGlobal * transfEstim;
        auto tgl = transfGlobal.linear().col(0);
        angles.push_back(atan2(tgl.y(), tgl.x()));
    }
}

void GraphSolverVfc::estimate(const std::vector<scan_overlap::Node> &nodes,
                              const scan_overlap::VectorTransform2 &odoms,
                              const std::vector<scan_overlap::Edge> &edges,
                              std::vector<double> &angles)
{

    scan_overlap::VFCRegistration vfc;
    scan_overlap::Transform2 transfEstim;
    scan_overlap::Transform2 transfGuess;
    scan_overlap::Transform2 transfGlobal;

    transfGlobal.setIdentity();
    angles.push_back(.0);
    for (int i = 1; i < nodes.size(); ++i)
    {
        const auto &nodeSrc = nodes.at(i - 1);
        const auto &nodeDst = nodes.at(i);
        transfGuess = odoms.at(i - 1).inverse() * odoms.at(i);
        vfc.setPointSetSrc(nodeSrc.cloud);
        vfc.setPointSetDst(nodeDst.cloud);
        vfc.computeRigidTransform(transfEstim, transfGuess);
        transfGlobal = transfGlobal * transfEstim;
        auto tgl = transfGlobal.linear().col(0);
        angles.push_back(atan2(tgl.y(), tgl.x()));
    }
}

void GraphSolverArs::estimate(const std::vector<scan_overlap::Node> &nodes,
                              const scan_overlap::VectorTransform2 &odoms,
                              const std::vector<scan_overlap::Edge> &edges,
                              std::vector<double> &angles)
{
    /*ars::FourierOptimizerBB1D fopt;
    double xtol_ = DEG2RAD(1.);
    fopt.setXTolerance(xtol_);
    fopt.enableXTolerance(true);
    fopt.enableYTolerance(false);

    angles.push_back(.0);
    for (int i = 1; i < nodes.size(); ++i)
    {
        const auto &nodeSrc = nodes.at(i - 1);
        const auto &nodeDst = nodes.at(i);
        std::vector<double> correlationFourier;
        ars::computeFourierCorr(nodeSrc.coeffs, nodeDst.coeffs, correlationFourier);
        fopt.setCoefficients(correlationFourier);

        double tMax, fLow, fUp;
        fopt.findGlobalMax(.0, M_PI, tMax, fLow, fUp);
        //angles.push_back(mod180(tMax + angles[i - 1]));
        angles.push_back(tMax);
        std::cout << RAD2DEG(tMax) << ",";
    }
    std::cout << std::endl;*/
    double xtol_ = DEG2RAD(.5);
    angles.push_back(.0);

    for (int i = 1; i < nodes.size(); ++i)
    {
        double tMax, fMax;
        const auto &nodeSrc = nodes.at(i - 1);
        const auto &nodeDst = nodes.at(i);
        std::vector<double> correlationFourier;
        ars::computeFourierCorr(nodeSrc.coeffs, nodeDst.coeffs, correlationFourier);
        ars::findGlobalMaxBBFourier(correlationFourier, .0, M_PI, xtol_, .0, tMax, fMax);
        angles.push_back(mod180(tMax + angles[i - 1]));
        std::cout << RAD2DEG(tMax) << ",";
    }
    std::cout << std::endl;
}

void GraphSolverArsGraph::estimate(const std::vector<scan_overlap::Node> &nodes,
                                   const scan_overlap::VectorTransform2 &odoms,
                                   const std::vector<scan_overlap::Edge> &edges,
                                   std::vector<double> &angles)
{
    ars::ArsGraph::Ptr graph(new ars::ArsGraph);
    ars::ArsGraphIntervalFull::Ptr interval(new ars::ArsGraphIntervalFull);
    graph->setFourierOrder(30);

    std::map<int, int> idToArsId;

    for (int i = 0; i < nodes.size(); ++i)
    {
        const auto &n = nodes.at(i);
        graph->addNode(n.coeffs);
        idToArsId[n.id] = i;
    }

    for (const auto &edge : edges)
    {
        int src = idToArsId[edge.src];
        int dst = idToArsId[edge.dst];
        graph->addEdgeWithDerivative(src, dst);
    }

    interval->initWithStationary(graph);
    ars::ArsGraphSolver solver(graph);

    std::vector<double> solution;
    double cost = 0;
    ars::ArsGraphSolver::Statistics stats;
    solver.solveWithStationary(solution, cost, stats, false);

    angles = solution;
}

double mod180(double angle){
    return (angle - floorf(angle / M_PI) * M_PI);
}