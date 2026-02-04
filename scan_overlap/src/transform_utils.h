#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>

namespace misc_tools {

using Vector3 = Eigen::Vector3d;
using Vector2 = Eigen::Vector2d;
using Quaternion = Eigen::Quaterniond;
using Transform3 = Eigen::Affine3d;
using Transform2 = Eigen::Affine2d;
using VectorTransform3 =
    std::vector<Transform3, Eigen::aligned_allocator<Transform3> >;
using VectorTransform2 =
    std::vector<Transform2, Eigen::aligned_allocator<Transform2> >;
using Scan = std::vector<double>;
using LaserSpecs = std::map<std::string, std::string>;
using Cloud = std::vector<Vector2>;

struct Node {
    int id;
    std::vector<int> adj;
    Node(int id_) : id(id_), adj() {};
    Node(int id_, std::vector<int> adj_) : id(id_), adj(adj_) {};
};
using Graph = std::vector<Node>;

struct ErrorData {
    int firstIdx;
    int lastIdx;
    double length;
    double errTransl;
    double errRot;
};
using VectorErrorData = std::vector<ErrorData>;

bool readTransformLine(std::istream& in, Transform3& transform);

bool readTimeTransformLine(std::istream& in,
                           double& time,
                           Transform3& transform);

bool readTimePosQuatLine(std::istream& in, double& time, Transform3& transform);

bool readTimePosQuatCovLine(std::istream& in, double& time, Transform3& transform);

bool readTimeRangesLine(std::istream& in, double& time, Scan& ranges);

bool readLaserSpecsLine(std::istream& in, std::string& key, std::string& val);

bool readTransformFile(const std::string& filename,
                       VectorTransform3& transforms);

bool readTimeTransformFile(const std::string& filename,
                           std::vector<double>& times,
                           VectorTransform3& transforms);

bool readTimePosQuatFile(const std::string& filename,
                         std::vector<double>& times,
                         VectorTransform3& transforms);

bool readTimePosQuatCovFile(const std::string& filename,
                         std::vector<double>& times,
                         VectorTransform3& transforms);

bool readTimeRangesFile(const std::string& filename,
                            std::vector<double>& times,
                            std::vector<Scan>& ranges);

bool readLaserSpecsFile(const std::string& filename,
                            LaserSpecs& laserSpecs);

void writePoseQuat(std::ostream& out,
                   const double& t,
                   const Transform3& pose);

void writePoseMat(std::ostream& out, const Transform3& pose);

void computeDistances(const VectorTransform3& transforms,
                      std::vector<double>& distances);

int findDistanceIdx(const std::vector<double>& distances,
                    double len,
                    double& factor);

int findDistanceIncrIdx(const std::vector<double>& distances,
                        int startIdx,
                        double lenStep,
                        double& factor);

void scanToCloud(const Scan& scan, const LaserSpecs& ls, Cloud& cloud);

void orderCloudRadially(Cloud& cloud);

double computeTranslationNorm(const Transform3& transformDelta);

double computeRotationAngle(const Transform3& transformDelta);

void interpolateTransform(const Transform3& transf0,
                          const Transform3& transf1,
                          double f,
                          Transform3& transfInterp);

bool findGtTransform(const double scanTs, 
                     const std::vector<double>& gtTimes, 
                     const VectorTransform3& gtTransforms,
                     Transform3& transform);

void trans3DToTrans2D(const Transform3& trans3D, Transform2& trans2D);

void fillCloud(const Cloud &cloud1, const Cloud &cloud2, Cloud &joined,
               double startAngle, double endAngle);

void computeErrors(const VectorTransform3& transformsRes,
                   const std::vector<double>& distancesRes,
                   const VectorTransform3& transformsGt,
                   const std::vector<double>& distancesGt,
                   double lenSeg,
                   double lenStep,
                   VectorErrorData& errors);

void computeErrorsInterp(const VectorTransform3& transformsRes,
                         const std::vector<double>& distancesRes,
                         const VectorTransform3& transformsGt,
                         const std::vector<double>& distancesGt,
                         double lenSeg,
                         double lenStep,
                         VectorErrorData& errors);

}  // namespace misc_tools

#endif
