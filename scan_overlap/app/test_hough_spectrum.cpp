#include <ars/HistogramCircularCorrelation.h>
#include <ars/Profiler.h>
#include <iostream>
#include <map>
#include <Eigen/Dense>

#include "transform_utils.h"
#include "simple_icp_registration.h"

#include <rofl/common/param_map.h>
#include <ars/HoughSpectrum.h>
#include <ars/utils.h>

#include <rofl/common/profiler.h>
#include <rofl/common/io.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

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

class GraphSolverHS : public GraphSolver
{
public:
    GraphSolverHS() = default;

    virtual ~GraphSolverHS() = default;

    virtual void estimate(const std::vector<scan_overlap::Node> &nodes,
                          const scan_overlap::VectorTransform2 &odoms,
                          const std::vector<scan_overlap::Edge> &edges,
                          std::vector<double> &angles) override;
};

int main(int argc, char **argv){
    // reading params and graph
    std::string filenameCfg, filenameGraph;
    bool enableHS, enableIcp;
    rofl::ParamMap params;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("in", filenameGraph, std::string(""));
    params.getParam<bool>("enable_hs", enableHS, true);
    params.getParam<bool>("enable_icp", enableIcp, false);

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n"
              << std::endl;

    std::vector<scan_overlap::Node> nodes;
    scan_overlap::VectorTransform2 gts, odoms;
    std::vector<scan_overlap::Edge> edges;
    std::vector<std::string> names;
    std::vector<std::vector<double>> solutions;

    scan_overlap::readGraph(filenameGraph, nodes, gts, odoms, edges);

    ROFL_VAR4(nodes.size(), gts.size(), odoms.size(), edges.size());

    /**
     * groundtruth elements
     */
    std::vector<double> anglesGT;
    auto gt0inv = gts.front().inverse();
    for (int i = 0; i < nodes.size(); ++i){
        auto gt = gt0inv * gts.at(i);
        anglesGT.push_back(atan2(gt.linear().col(0).y(), gt.linear().col(0).x()));
    }

    /**
     * ICP
     */

    std::vector<double> anglesIcp;
    if (enableIcp)
    {
        GraphSolverIcp solverIcp;
        solverIcp.estimate(nodes, odoms, edges, anglesIcp);
        solutions.push_back(anglesIcp);
        names.push_back("icp");
    }

    /**
     * HS
     */

    std::vector<double> anglesHS;
    if (enableHS)
    {
        GraphSolverHS solverHS;
        solverHS.estimate(nodes, odoms, edges, anglesHS);
        //find  if the correct ars angle is tMax or tMax - 180
        for(int i = 1; i < anglesHS.size(); i++){
            double tGT = anglesGT.at(i);
            double& t = anglesHS.at(i);
            double t2 = t - M_PI;
            //swap if t - 180 is closer to GT than t
            if(abs(tGT-t2) < abs(tGT - t))
                t = t2;
        }

        solutions.push_back(anglesHS);
        names.push_back("hs");
    }

     /**
     * Gathering results
     */
    std::string methodsEnabled = "";
    if (enableIcp)
        methodsEnabled += "icp_";
    if (enableHS)
        methodsEnabled += "hs_";
    ROFL_VAR1(methodsEnabled);
    std::string filenameOut = rofl::generateStampedString("results_" + methodsEnabled, ".csv");
    ROFL_VAR1(filenameOut);
    std::stringstream fileOut;
    fileOut << "id,\tgt,\t\todom,";
    if (enableIcp)
        fileOut << "\t\ticp,";
    if (enableHS)
        fileOut << "\t\ths,";
    fileOut << "\n";

    auto odom0inv = odoms.front().inverse();
    for (int i = 0; i < nodes.size(); ++i)
    {
        auto odom = odom0inv * odoms.at(i);
        fileOut << std::fixed << std::setprecision(5)
                << nodes.at(i).id << ",\t" << RAD2DEG(anglesGT.at(i))
                << ",\t" << RAD2DEG(atan2(odom.linear().col(0).y(), odom.linear().col(0).x())) << ",\t";
        if (enableIcp)
            if (i < anglesIcp.size())
                fileOut << RAD2DEG(anglesIcp.at(i)) << ",\t";
        if (enableHS)
            if (i < anglesHS.size())
                fileOut << RAD2DEG(anglesHS.at(i)) << ",\t";
        fileOut << "\n";
    }
    std::cout << fileOut.str() << std::endl;

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

void GraphSolverHS::estimate(const std::vector<scan_overlap::Node> &nodes,
                             const scan_overlap::VectorTransform2 &odoms,
                             const std::vector<scan_overlap::Edge> &edges,
                             std::vector<double> &angles){
    angles.push_back(.0);
    double hsSrcExecTime, hsDstExecTime;
    ars::HoughSpectrum hs;
    double thetaStep = DEG2RAD(.5);
    double rhoStep = 1.0;
    double rhoMax = 200.0;

    hs.init(thetaStep, rhoStep, rhoMax);

    for(int i = 1; i < nodes.size(); i++){
        const auto& pointsSrc = nodes[i].cloud;
        const auto& pointsDst = nodes[i-1].cloud;
        // Computes Hough spectra
    {
        ars::ScopedTimer timer("HoughSpectrum");
        hs.insertPoint(pointsSrc.begin(), pointsSrc.end());
        hsSrcExecTime = timer.elapsedTimeMs();
    }
    Eigen::VectorXd spectrumSrc = hs.spectrum();
    {
        ars::ScopedTimer timer("HoughSpectrum");
        hs.insertPoint(pointsDst.begin(), pointsDst.end());
        hsDstExecTime = timer.elapsedTimeMs();
    }
    Eigen::VectorXd spectrumDst = hs.spectrum();

    // Computes correlation of spectra
    {
        ars::ScopedTimer("HoughCorrelation");
        ars::HistogramCircularCorrelation hcc;
        double corrTmp;
        double corrMax = 0.0;
        int imax = 0;
        for (int k = 0; k < spectrumSrc.size(); ++k) {
            hcc.computeHistogramCorrelation(spectrumSrc, spectrumDst, k, corrTmp);
            if (corrTmp > corrMax) {
                imax = k;
                corrMax = corrTmp;
            }
        }
        double rot = thetaStep * imax;
        angles.push_back(scan_overlap::mod180(rot + angles[i - 1]));
    }
    }
}