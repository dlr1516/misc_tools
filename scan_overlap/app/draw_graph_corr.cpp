#include <fstream>
#include <iostream>
#include <thread>

#include <pcl/console/print.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/pcl_plotter.h>

#include <vtkCallbackCommand.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

#include <rofl/common/param_map.h>

#include <ars/ArsGraph.h>
#include <ars/ArsGraphSolver.h>
#include <ars/ars2d.h>
#include <ars/utils.h>

#include "transform_utils.h"

struct CorrelationPlot {
    int isrc;
    int idst;
    std::vector<double> angles;
    std::vector<double> values;
};
std::vector<CorrelationPlot> correlationPlots;

// Control flags used by visualizer
std::atomic<int> current_plot(0);
std::atomic<bool> update_plot(true);
std::atomic<bool> running(true);

pcl::visualization::PCLPlotter::Ptr plotter;

void keyboardEventOccurred(vtkObject* caller,
                           unsigned long eventId,
                           void* clientData,
                           void* callData);

int main(int argc, char* argv[]) {
    std::string filenameCfg, filenameGraph;
    int fourierOrder, angleNum;
    double sigma;
    rofl::ParamMap params;

    // Reads params from command line
    params.read(argc, argv);
    params.getParam<std::string>("cfg", filenameCfg, std::string(""));
    params.read(filenameCfg);
    params.read(argc, argv);
    params.getParam<std::string>("in", filenameGraph, std::string(""));
    params.getParam<int>("fourierOrder", fourierOrder, 30);
    params.getParam<int>("angleNum", angleNum, 360);
    params.getParam<double>("sigma", sigma, 0.05);

    std::cout << "\nParams:" << std::endl;
    params.write(std::cout);
    std::cout << "-------\n" << std::endl;

    std::vector<scan_overlap::Node> nodes;
    scan_overlap::VectorTransform2 gts, odoms;
    std::vector<scan_overlap::Edge> edges;

    scan_overlap::readGraph(filenameGraph, nodes, gts, odoms, edges);

    // Computation of ARS coefficients for each node
    ars::AngularRadonSpectrum2d arsSrc;
    arsSrc.setARSFOrder(fourierOrder);
    arsSrc.initLUT(0.0001);
    arsSrc.setComputeMode(ars::ArsKernelIsotropic2d::ComputeMode::PNEBI_LUT);
    std::map<int, int> indexToPos;
    int pos = 0;
    for (auto& n : nodes) {
        std::cout << "Node " << n.id << ", cloud size " << n.cloud.size()
                  << ", computing ARS" << std::endl;

        arsSrc.insertIsotropicGaussians(n.cloud, sigma);
        for (int i = 0; i < arsSrc.coefficients().size(); ++i) {
            std::cout << "\t" << i << " \t" << arsSrc.coefficients().at(i)
                      << "\n";
        }
        std::cout << std::endl;

        n.setCoeffs(arsSrc.coefficients());
        indexToPos.insert(std::make_pair(n.id, pos));
        pos++;
    }

    for (auto& e : edges) {
        if (indexToPos.find(e.src) == indexToPos.end() ||
            indexToPos.find(e.dst) == indexToPos.end()) {
            std::cerr << "Edge (" << e.src << ", " << e.dst
                      << ") has invalid node indices." << std::endl;
            continue;
        }
        std::cout << "Edge (" << e.src << ", " << e.dst << ") -> nodes ("
                  << indexToPos[e.src] << ", " << indexToPos[e.dst] << ")"
                  << std::endl;
        const auto& nodeSrc = nodes[indexToPos[e.src]];
        const auto& nodeDst = nodes[indexToPos[e.dst]];
        std::vector<double> correlationFourier;
        ars::computeFourierCorr(nodeSrc.coeffs, nodeDst.coeffs,
                                correlationFourier);

        CorrelationPlot plot;
        plot.isrc = e.src;
        plot.idst = e.dst;
        for (int i = 0; i < angleNum; ++i) {
            plot.angles.push_back(180.0 * i / angleNum);
            plot.values.push_back(ars::evaluateFourier(
                correlationFourier, 2.0 * M_PI * i / angleNum));
        }
        correlationPlots.push_back(plot);
    }
    std::cout << "Computed correlation for " << correlationPlots.size()
              << " edges." << std::endl;

    // Display correlation plot
    plotter = std::make_shared<pcl::visualization::PCLPlotter>();
    plotter->setWindowName("ARS Correlation Plot");
    plotter->setXTitle("angle [deg]");
    plotter->setYTitle("correlation");

    // Set up keyboard callback usng VTK as required by PCLPlotter
    vtkSmartPointer<vtkCallbackCommand> keypressCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    keypressCallback->SetCallback(keyboardEventOccurred);
    // plotter.getInteractor()->AddObserver(vtkCommand::KeyPressEvent,
    //                                      keypressCallback);
    // plotter->setViewInteractor(keypressCallback);
    plotter->getRenderWindow()->GetInteractor()->AddObserver(
        vtkCommand::KeyPressEvent, keypressCallback);

    current_plot = 0;
    plotter->addPlotData(correlationPlots[current_plot].angles,
                         correlationPlots[current_plot].values);
    std::stringstream ss;
    ss << "Correlation between node " << correlationPlots[current_plot].isrc
       << " and node " << correlationPlots[current_plot].idst;
    plotter->setTitle(ss.str().c_str());
    plotter->spin();

    // running = true;
    // while (running) {
    //     if (update_plot) {
    //         plotter->addPlotData(correlationPlots[current_plot].angles,
    //                              correlationPlots[current_plot].values);
    //         ss.str("");
    //         ss << "Correlation between node "
    //            << correlationPlots[current_plot].isrc << " and node "
    //            << correlationPlots[current_plot].idst;
    //         plotter->setTitle(ss.str().c_str());
    //         update_plot = false;
    //     }
    //     plotter->spinOnce(100);
    // }

    return 0;
}

// void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
//                            void*) {
//     const int N = (int)correlationPlots.size();
//     if (event.keyDown()) {
//         if (event.getKeySym() == "n") {
//             current_plot = (current_plot + 1) % N;
//             update_plot = true;
//         } else if (event.getKeySym() == "p") {
//             current_plot = (current_plot + N - 1) % N;
//             update_plot = true;
//         } else if (event.getKeySym() == "q") {
//             running = false;
//         }
//     }
// }

void keyboardEventOccurred(vtkObject* caller,
                           unsigned long eventId,
                           void* clientData,
                           void* callData) {
    auto interactor = static_cast<vtkRenderWindowInteractor*>(caller);
    std::string key = interactor->GetKeySym();
    std::cout << "Key pressed: " << key << std::endl;

    const int N = (int)correlationPlots.size();
    std::stringstream ss;
    if (key == "Right" || key == "Left") {
        if (key == "Right") {
            std::cout << "Next plot\n";
            current_plot = (current_plot + 1) % N;
        }
        if (key == "Left") {
            std::cout << "Previous plot\n";
            current_plot = (current_plot + N - 1) % N;
        }

        plotter->clearPlots();
        plotter->addPlotData(correlationPlots[current_plot].angles,
                             correlationPlots[current_plot].values);
        ss << "Correlation between node " << correlationPlots[current_plot].isrc
           << " and node " << correlationPlots[current_plot].idst;
        plotter->setTitle(ss.str().c_str());
    }
}