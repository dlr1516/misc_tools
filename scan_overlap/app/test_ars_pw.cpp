#include <iostream>
#include <vector>

#include <ars/ars2d.h>
#include <ars/utils.h>
#include <ars/ArsGraph.h>
#include <ars/ArsGraphSolver.h>

#include "types.h"
#include "transform_utils.h"

#include <pcl/point_types.h>

int main(int argc, char **argv)
{
    std::vector<scan_overlap::Node> nodes;
    scan_overlap::Node n(0);

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

    int i, j;
    double tMax, fMax;
    const auto &nodeSrc = nodes.at(i - 1);
    const auto &nodeDst = nodes.at(i);
    std::vector<double> correlationFourier;
    ars::computeFourierCorr(nodeDst.coeffs, nodeSrc.coeffs, correlationFourier);
    double xtol_ = DEG2RAD(1.);
    ars::findGlobalMaxBBFourier(correlationFourier, .0, M_PI, xtol_, .0, tMax, fMax);
    std::vector<double> angles;
    angles.push_back(scan_overlap::mod180(tMax + angles[i - 1]));
    std::cout << RAD2DEG(tMax) << ",";

    return 0;
}