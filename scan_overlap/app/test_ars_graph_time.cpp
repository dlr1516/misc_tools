#include <Eigen/Dense>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>

#include <rofl/common/param_map.h>
#include <rofl/common/profiler.h>

#include <ars/ArsGraph.h>
#include <ars/ArsGraphSolver.h>
#include <ars/ars2d.h>
#include <ars/utils.h>

#include "transform_utils.h"

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.017453293)
#endif

void findSubGraph(const std::vector<scan_overlap::Node>& nodes,
                  const scan_overlap::VectorTransform2& gts,
                  const scan_overlap::VectorTransform2& odoms,
                  const std::vector<scan_overlap::Edge>& edges,
                  int nodesNum,
                  // int permutationId,
                  std::vector<scan_overlap::Node>& subNodes,
                  scan_overlap::VectorTransform2& subGts,
                  scan_overlap::VectorTransform2& subOdoms,
                  std::vector<scan_overlap::Edge>& subEdges);

void computeNodeArs(std::vector<scan_overlap::Node>& nodes);

void estimateArsGraph(const std::vector<scan_overlap::Node>& nodes,
                      const scan_overlap::VectorTransform2& odoms,
                      const std::vector<scan_overlap::Edge>& edges,
                      std::vector<double>& angles);

int main(int argc, char** argv) {
    // reading params and graph
    std::string filenameCfg, filenameGraph, dirnameGraph;
    std::vector<std::string> filenames;
    bool batch;
    rofl::ParamMap params;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("in", filenameGraph, std::string(""));
    params.getParam<std::string>("in_dir", dirnameGraph, std::string(""));
    params.getParam<bool>("batch", batch, false);

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    if (batch) {
        const std::filesystem::path path{dirnameGraph};
        for (auto& entry : std::filesystem::directory_iterator(path)) {
            filenames.push_back(entry.path());
        }
    } else
        filenames.push_back(filenameGraph);

    for (const auto& filename : filenames) {
        filenameGraph = filename;

        std::vector<scan_overlap::Node> nodes;
        scan_overlap::VectorTransform2 gts, odoms;
        std::vector<scan_overlap::Edge> edges;
        std::vector<std::string> names;
        std::vector<std::vector<double>> solutions;

        scan_overlap::readGraph(filenameGraph, nodes, gts, odoms, edges);
        computeNodeArs(nodes);

        // Solves the subgraph with ARS and computes the times
        for (int i = 2; i < nodes.size(); ++i) {
            std::vector<scan_overlap::Node> subNodes;
            scan_overlap::VectorTransform2 subGts;
            scan_overlap::VectorTransform2 subOdoms;
            std::vector<scan_overlap::Edge> subEdges;
            findSubGraph(nodes, gts, odoms, edges, i, subNodes, subGts,
                         subOdoms, subEdges);

            std::vector<double> angles;
            estimateArsGraph(subNodes, subOdoms, subEdges, angles);
        }
        rofl::Profiler::getProfiler().printStats(std::cout);
    }

    std::cout << "\n\n-----\nFinal stats:" << std::endl;
    rofl::Profiler::getProfiler().printStats(std::cout);

    return 0;
}

void findSubGraph(const std::vector<scan_overlap::Node>& nodes,
                  const scan_overlap::VectorTransform2& gts,
                  const scan_overlap::VectorTransform2& odoms,
                  const std::vector<scan_overlap::Edge>& edges,
                  int nodesNum,
                  // int permutationId,
                  std::vector<scan_overlap::Node>& subNodes,
                  scan_overlap::VectorTransform2& subGts,
                  scan_overlap::VectorTransform2& subOdoms,
                  std::vector<scan_overlap::Edge>& subEdges) {
    // Selects a subset of nodes by chosing the permutation permutationId
    // for (int i = 0; i < permutationId; ++i) {
    //     std::next_permutation(subNodes.begin(), subNodes.end());
    // }
    subNodes.insert(subNodes.begin(), nodes.begin(), nodes.begin() + nodesNum);
    std::map<int, int> idToSubId;
    for (int i = 0; i < subNodes.size(); ++i) {
        idToSubId[subNodes.at(i).id] = i;
    }

    // Selects the corresponding GT and odom transforms
    subGts.clear();
    subOdoms.clear();
    for (const auto& n : subNodes) {
        if (idToSubId.find(n.id) == idToSubId.end()) {
            std::cerr << "Node " << n.id << " not found in subgraph."
                      << std::endl;
            continue;
        }
        int subId = idToSubId[n.id];
        subGts.push_back(gts[subId]);
        subOdoms.push_back(odoms[subId]);
    }

    // Selects only the edges between the selected nodes
    subEdges.clear();
    for (const auto& e : edges) {
        if (idToSubId.find(e.src) == idToSubId.end() ||
            idToSubId.find(e.dst) == idToSubId.end()) {
            continue;
        }
        subEdges.push_back(e);
    }
}

void computeNodeArs(std::vector<scan_overlap::Node>& nodes) {
    for (auto& n : nodes) {
        std::cout << "Node " << n.id << ", cloud size " << n.cloud.size()
                  << std::endl;

        ars::AngularRadonSpectrum2d arsSrc;

        int fourierOrder =
            30;  // TODO: make ARS params configurable from file/command line
        double sigma = 0.05;

        arsSrc.setARSFOrder(fourierOrder);

        arsSrc.initLUT(0.0001);
        arsSrc.setComputeMode(
            ars::ArsKernelIsotropic2d::ComputeMode::PNEBI_LUT);

        ars::VectorVector2 acesPoints1;
        acesPoints1.push_back(n.cloud.front());
        for (auto& p : n.cloud) {
            // if(scan_overlap::squaredDistance2D(p, acesPoints1.back()) >
            // 0.2*0.2)
            acesPoints1.push_back(ars::Vector2(p.x(), p.y()));
        }
        arsSrc.insertIsotropicGaussians(acesPoints1, sigma);

        std::cout << "ars.coefficients().at(0) " << arsSrc.coefficients().at(0)
                  << ", ars.coefficients().at(2) "
                  << arsSrc.coefficients().at(2) << std::endl;

        std::cout << "\n------\n" << std::endl;

        std::cout << "\nARS Coefficients:\n";
        std::cout << "\ti \tLUT\n";
        for (int i = 0; i < arsSrc.coefficients().size(); ++i) {
            std::cout << "\t" << i << " \t" << arsSrc.coefficients().at(i)
                      << "\n";
        }
        std::cout << std::endl;

        n.setCoeffs(arsSrc.coefficients());
    }
}

void estimateArsGraph(const std::vector<scan_overlap::Node>& nodes,
                      const scan_overlap::VectorTransform2& odoms,
                      const std::vector<scan_overlap::Edge>& edges,
                      std::vector<double>& angles) {
    ars::ArsGraph::Ptr graph(new ars::ArsGraph);
    ars::ArsGraphIntervalFull::Ptr interval(new ars::ArsGraphIntervalFull);
    graph->setFourierOrder(30);
    double xtol = DEG2RAD(1.);

    std::map<int, int> idToArsId;

    for (int i = 0; i < nodes.size(); ++i) {
        const auto& n = nodes.at(i);
        graph->addNode(n.coeffs);
        idToArsId[n.id] = i;
    }

    for (const auto& edge : edges) {
        int src = idToArsId[edge.src];
        int dst = idToArsId[edge.dst];
        graph->addEdgeWithDerivative(src, dst);
    }

    interval->initWithStationary(graph);
    ars::ArsGraphSolver solver(graph, xtol);

    std::vector<double> solution;
    double cost = 0;
    ars::ArsGraphSolver::Statistics stats;

    {
        std::stringstream ss;
        ss << "ars_graph_time_nodes_" << nodes.size();
        rofl::ScopedTimer timer(ss.str());
        solver.solveWithStationary(solution, cost, stats, false);
    }

    for (const auto& a : solution)
        angles.push_back(scan_overlap::mod180(-a));

    // angles = solution;
}