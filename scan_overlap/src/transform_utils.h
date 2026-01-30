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

using Vector3 = Eigen::Vector3f;
using Vector2 = Eigen::Vector2f;
using Quaternion = Eigen::Quaternionf;
using Transform3 = Eigen::Affine3f;
using Transform2 = Eigen::Affine2f;
using VectorTransform3 =
    std::vector<Transform3, Eigen::aligned_allocator<Transform3> >;
using Scan = std::vector<float>;
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
    float length;
    float errTransl;
    float errRot;
};
using VectorErrorData = std::vector<ErrorData>;

bool readTransformLine(std::istream& in, Transform3& transform);

bool readTimeTransformLine(std::istream& in,
                           float& time,
                           Transform3& transform);

bool readTimePosQuatLine(std::istream& in, double& time, Transform3& transform);

bool readTimeRangesLine(std::istream& in, double& time, Scan& ranges);

bool readLaserSpecsLine(std::istream& in, std::string& key, std::string& val);

bool readTransformFile(const std::string& filename,
                       VectorTransform3& transforms);

bool readTimeTransformFile(const std::string& filename,
                           std::vector<float>& times,
                           VectorTransform3& transforms);

bool readTimePosQuatFile(const std::string& filename,
                         std::vector<double>& times,
                         VectorTransform3& transforms);

bool readTimeRangesFile(const std::string& filename,
                            std::vector<double>& times,
                            std::vector<Scan>& ranges);

bool readLaserSpecsFile(const std::string& filename,
                            LaserSpecs& laserSpecs);

void writePoseQuat(std::ostream& out,
                   const float& t,
                   const Eigen::Affine3f& pose);

void writePoseMat(std::ostream& out, const Eigen::Affine3f& pose);

void computeDistances(const VectorTransform3& transforms,
                      std::vector<float>& distances);

int findDistanceIdx(const std::vector<float>& distances,
                    float len,
                    float& factor);

int findDistanceIncrIdx(const std::vector<float>& distances,
                        int startIdx,
                        float lenStep);

void scanToCloud(const Scan& scan, const LaserSpecs& ls, Cloud& cloud);

void orderCloudRadially(Cloud& cloud);

float computeTranslationNorm(const Transform3& transformDelta);

float computeRotationAngle(const Transform3& transformDelta);

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
                   const std::vector<float>& distancesRes,
                   const VectorTransform3& transformsGt,
                   const std::vector<float>& distancesGt,
                   float lenSeg,
                   float lenStep,
                   VectorErrorData& errors);

void computeErrorsInterp(const VectorTransform3& transformsRes,
                         const std::vector<float>& distancesRes,
                         const VectorTransform3& transformsGt,
                         const std::vector<float>& distancesGt,
                         float lenSeg,
                         float lenStep,
                         VectorErrorData& errors);

}  // namespace misc_tools

#endif
