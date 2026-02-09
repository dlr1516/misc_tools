#include "transform_utils.h"

namespace scan_overlap
{

    bool readTransformLine(std::istream &in, Transform3 &transform)
    {
        std::array<double, 12> values;
        if (in >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >>
            values[5] >> values[6] >> values[7] >> values[8] >> values[9] >>
            values[10] >> values[11])
        {
            transform.matrix() << values[0], values[1], values[2], values[3],
                values[4], values[5], values[6], values[7], values[8], values[9],
                values[10], values[11];
            // std::cout << "transformation matrix:\n" << transform.matrix() <<
            // std::endl;
            return true;
        }
        else
        {
            std::cerr
                << "Cannot read transform matrix from line: 12 floats required"
                << std::endl;
            return false;
        }
    }

    bool readTimeTransformLine(std::istream &in,
                               double &time,
                               Transform3 &transform)
    {
        if ((in >> time) && readTransformLine(in, transform))
        {
            return true;
        }
        else
        {
            std::cerr
                << "Cannot read line: invalid time stamp or transformation matrix"
                << std::endl;
            return false;
        }
    }

    bool readTimePosQuatLine(std::istream &in, double &time, Transform3 &transform)
    {
        double x, y, z, qx, qy, qz, qw;
        if (in >> time >> x >> y >> z >> qx >> qy >> qz >> qw)
        {
            Quaternion q(qw, qx, qy, qz);
            Eigen::Translation3d v(x, y, z);
            transform = v * q;
            return true;
        }
        else
        {
            std::cerr << "Cannot read line in format: time x y z qx qy qz qw"
                      << std::endl;
            return false;
        }
    }

    bool readTimePosQuatCovLine(std::istream &in, double &time, Transform3 &transform)
    {
        double x, y, z, qx, qy, qz, qw;
        std::string line;
        std::getline(in, line);
        std::stringstream ls(line);
        if (ls >> time >> x >> y >> z >> qx >> qy >> qz >> qw)
        {
            Quaternion q(qw, qx, qy, qz);
            Eigen::Translation3d v(x, y, z);
            transform = v * q;
            return true;
        }
        else
        {
            std::cerr << "Cannot read line in format: time x y z qx qy qz qw"
                      << std::endl;
            return false;
        }
    }

    bool readTimeRangesLine(std::istream &in, double &time, Scan &ranges)
    {
        double r;
        std::string line;
        std::getline(in, line);
        std::stringstream ls(line);
        /*if (!(in >> time)) {
            std::cerr << "Cannot read line in format: time ranges"
                      << std::endl;
            return false;
        }
        while((in.peek()!='\n') && in >> r) ranges.push_back(r);
        return true;*/
        if (!(ls >> time))
        {
            std::cerr << "Cannot read line in format: time ranges"
                      << std::endl;
            return false;
        }
        while (ls >> r)
            ranges.push_back(r);
        return true;
    }

    bool readLaserSpecsLine(std::istream &in, std::string &key, std::string &val)
    {
        double r;
        if (!(in >> key >> val))
        {
            std::cerr << "Cannot read line in format: key value"
                      << std::endl;
            return false;
        }
        // remove ":" char from yaml
        key.pop_back();
        return true;
    }

    bool readCalibLine(std::istream &in,
                       std::string &label,
                       Transform3 &transform)
    {
        if ((in >> label) && readTransformLine(in, transform))
        {
            return true;
        }
        else
        {
            std::cerr << "Cannot read line: invalid calibration data label or "
                         "transformation matrix"
                      << std::endl;
            return false;
        }
    }

    bool readTransformFile(const std::string &filename,
                           VectorTransform3 &transforms)
    {
        Transform3 tr;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open transformation file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readTransformLine(file, tr))
        {
            transforms.push_back(tr);
        }
        file.close();
        return true;
    }

    bool readTimeTransformFile(const std::string &filename,
                               std::vector<double> &times,
                               VectorTransform3 &transforms)
    {
        Transform3 tr;
        double t;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open transformation file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readTimeTransformLine(file, t, tr))
        {
            times.push_back(t);
            transforms.push_back(tr);
        }
        file.close();
        return true;
    }

    bool readTimePosQuatFile(const std::string &filename,
                             std::vector<double> &times,
                             VectorTransform3 &transforms)
    {
        Transform3 tr;
        double t;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open transformation file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readTimePosQuatLine(file, t, tr))
        {
            times.push_back(t);
            transforms.push_back(tr);
        }
        file.close();
        return true;
    }

    bool readTimePosQuatCovFile(const std::string &filename, std::vector<double> &times, VectorTransform3 &transforms)
    {
        Transform3 tr;
        double t;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open transformation file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readTimePosQuatCovLine(file, t, tr))
        {
            times.push_back(t);
            transforms.push_back(tr);
        }
        file.close();
        return true;
    }

    bool readTimeRangesFile(const std::string &filename, std::vector<double> &times, std::vector<Scan> &ranges)
    {
        Scan r;
        double t;
        long line = 0;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open ranges file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readTimeRangesLine(file, t, r))
        {
            times.push_back(t);
            ranges.push_back(r);
            // std::cout << line++ <<" scan size: " << r.size() << std::endl;
            r.clear();
        }
        file.close();
        return true;
    }

    bool readLaserSpecsFile(const std::string &filename, LaserSpecs &laserSpecs)
    {
        std::string k;
        std::string v;
        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open laser parameters file \"" << filename << "\""
                      << std::endl;
            return false;
        }
        while (readLaserSpecsLine(file, k, v))
        {
            laserSpecs[k] = v;
        }
        file.close();
        return true;
    }

    void writePoseQuat(std::ostream &out,
                       const double &t,
                       Transform3 &pose)
    {
        Vector3 transl = pose.translation();
        Quaternion quat(
            pose.rotation()); // Eigen coefficients order [x, y, z, w]
        out << t << " " << transl(0) << " " << transl(1) << " " << transl(2) << " "
            << quat.coeffs()(0) << " " << quat.coeffs()(1) << " "
            << quat.coeffs()(2) << " " << quat.coeffs()(3) << "\n";
    }

    void writePoseMat(std::ostream &out, const Transform3 &pose)
    {
        out << pose.matrix()(0, 0) << " " << pose.matrix()(0, 1) << " "
            << pose.matrix()(0, 2) << " " << pose.matrix()(0, 3) << " "
            << pose.matrix()(1, 0) << " " << pose.matrix()(1, 1) << " "
            << pose.matrix()(1, 2) << " " << pose.matrix()(1, 3) << " "
            << pose.matrix()(2, 0) << " " << pose.matrix()(2, 1) << " "
            << pose.matrix()(2, 2) << " " << pose.matrix()(2, 3) << " "
            << "\n";
    }

    void computeDistances(const VectorTransform3 &transforms,
                          std::vector<double> &distances)
    {
        if (transforms.empty())
        {
            return;
        }
        distances.resize(transforms.size());
        distances[0] = 0.0f;
        for (int i = 1; i < (int)distances.size(); ++i)
        {
            distances[i] =
                distances[i - 1] +
                (transforms[i - 1].inverse() * transforms[i]).translation().norm();
        }
    }

    int findDistanceIdx(const std::vector<double> &distances,
                        double len,
                        double &factor)
    {
        auto it = lower_bound(distances.begin(), distances.end(), len);
        factor = 0.0;
        if (it == distances.end())
        {
            factor = 0.0;
            return distances.size();
        }
        else if (it == distances.begin())
        {
            factor = 0.0;
            return 0;
        }
        auto itPrev = it - 1;
        factor = (len - *itPrev) / (*it - *itPrev);
        //		std::cout << "len " << len << ": samples "
        //				<< (int)std::distance(distances.begin(), itPrev)
        //<< " at distance " << *itPrev << " and "
        //				<< (int)std::distance(distances.begin(), it) <<
        //" at distance " << *it
        //				<< " factor " << factor << ": distPrev * (1 - "
        //<< factor << ") + distCurr * " << factor << " = "
        //				<< (*itPrev * (1- factor) + *it * factor)
        //				<< std::endl;
        return (int)std::distance(distances.begin(), itPrev);
    }

    int findDistanceIncrIdx(const std::vector<double> &distances,
                            int startIdx,
                            double lenStep,
                            double &factor)
    {
        if (startIdx < 0 || startIdx >= (int)distances.size())
        {
            return distances.size();
        }
        double len = distances[startIdx] + lenStep;
        auto it = lower_bound(distances.begin() + startIdx, distances.end(), len);
        factor = 0.0;
        if (it != distances.begin())
        {
            auto itPrev = it - 1;
            factor = (*it - len) / (*it - *itPrev);
        }
        return (int)std::distance(distances.begin(), it);
    }

    void scanToCloud(const Scan &scan, const LaserSpecs &ls, Cloud &cloud)
    {
        double inc = stod(ls.at("angle_increment"));
        double angle = stod(ls.at("angle_min"));
        double rangeMin = stod(ls.at("range_min"));
        double rangeMax = stod(ls.at("range_max"));
        for (const auto &range : scan)
        {
            if (range > rangeMin && range < rangeMax)
            {
                Vector2 point(range * cos(angle), range * sin(angle));
                cloud.push_back(point);
            }
            angle += inc;
        }
    }

    void orderCloudRadially(Cloud &cloud)
    {
        std::sort(cloud.begin(), cloud.end(),
                  [](const Vector2 &a, Vector2 &b)
                  {
                      double angleA = atan2(a.y(), a.x());
                      double angleB = atan2(b.y(), b.x());
                      return angleA < angleB;
                  });
    }

    double computeTranslationNorm(const Transform3 &transformDelta)
    {
        return transformDelta.translation().norm();
    }

    double computeRotationAngle(const Transform3 &transformDelta)
    {
        double m00 = transformDelta.matrix()(0, 0);
        double m11 = transformDelta.matrix()(1, 1);
        double m22 = transformDelta.matrix()(2, 2);
        double delta = 0.5 * (m00 + m11 + m22 - 1.0);
        return acos(std::max(std::min(delta, 1.0), -1.0));
    }

    void interpolateTransform(const Transform3 &transf0,
                              const Transform3 &transf1,
                              double f,
                              Transform3 &transfInterp)
    {
        if (f < 0.0f || f > 1.0f)
        {
            std::cerr << __FILE__ << "," << __LINE__
                      << ": invalid interpolation factor " << f
                      << ": must be between 0.0 and 1.0" << std::endl;
            return;
        }
        Quaternion rot0(transf0.linear());
        Quaternion rot1(transf1.linear());
        Vector3 transl0 = transf0.translation();
        Vector3 transl1 = transf1.translation();
        transfInterp.translation() = (1.0 - f) * transl0 + f * transl1;
        transfInterp.linear() = rot0.slerp(f, rot1).toRotationMatrix();
    }

    bool findGtTransform(const double scanTs,
                         const std::vector<double> &gtTimes,
                         const VectorTransform3 &gtTransforms,
                         Transform3 &transform)
    {
        for (int i = 1; i < gtTimes.size(); i++)
        {
            double tsNext = gtTimes[i];
            if (scanTs <= tsNext)
            {
                double tsPrev = gtTimes[i - 1];
                Transform3 next = gtTransforms[i];
                Transform3 prev = gtTransforms[i - 1];
                double f = (scanTs - tsPrev) / (tsNext - tsPrev);
                interpolateTransform(prev, next, f, transform);
                return true;
            }
        }
        return false;
    }

    void trans3DToTrans2D(const Transform3 &trans3D, Transform2 &trans2D)
    {
        trans2D.linear() = trans3D.linear().topLeftCorner<2, 2>();
        trans2D.translation() = trans3D.translation().topRows<2>();
    }

    void fillCloud(const Cloud &cloud1, const Cloud &cloud2, Cloud &joined,
                   double startAngle, double endAngle)
    {
        joined.clear();
        joined = cloud1;
        startAngle += (M_PI / 180.);
        endAngle -= (M_PI / 360.);

        if (startAngle < endAngle)
        { // fill interval doesn't go over pi/-pi
            for (auto &p : cloud2)
            {
                double angle = atan2(p.y(), p.x());
                if (angle > startAngle && angle < endAngle)
                    joined.push_back(p);
            }
        }
        else
        {
            for (auto &p : cloud2)
            {
                double angle = atan2(p.y(), p.x());
                if ((startAngle < angle && angle <= M_PI) ||
                    (-M_PI <= angle && angle < endAngle))
                    joined.push_back(p);
            }
        }
    }

    void computeErrors(const VectorTransform3 &transformsRes,
                       const std::vector<double> &distancesRes,
                       const VectorTransform3 &transformsGt,
                       const std::vector<double> &distancesGt,
                       double lenSeg,
                       double lenStep,
                       VectorErrorData &errors)
    {
        ErrorData err;
        Transform3 transformDeltaRes, transformDeltaGt, transformError;
        int numRes, numGt, firstIdxGt, lastIdxGt, firstIdxRes, lastIdxRes;
        double factorRes, factorGt;
        // float len;

        if (transformsRes.size() != distancesRes.size() ||
            transformsGt.size() != distancesGt.size())
        {
            std::cout << "Error: different size of distances and trasforms"
                      << std::endl;
            return;
        }
        numRes = transformsRes.size();
        numGt = transformsGt.size();

        firstIdxRes = 0;
        firstIdxGt = 0;
        // len = 0.0f;
        while (firstIdxRes < numRes && firstIdxGt < numGt)
        {
            // len += lenIncr;
            // lastIdxRes = findDistanceIdx(distancesRes, lenSeg);
            // lastIdxGt = findDistanceIdx(distancesGt, lenSeg);
            lastIdxRes =
                findDistanceIncrIdx(distancesRes, firstIdxRes, lenSeg, factorRes);
            lastIdxGt =
                findDistanceIncrIdx(distancesGt, firstIdxGt, lenSeg, factorGt);
            if (lastIdxRes >= numRes || lastIdxGt >= numGt)
            {
                break;
            }
            transformDeltaRes =
                transformsRes[firstIdxRes].inverse() * transformsRes[lastIdxRes];
            transformDeltaGt =
                transformsGt[firstIdxGt].inverse() * transformsGt[lastIdxGt];
            transformError = transformDeltaGt.inverse() * transformDeltaRes;
            // Computes the error
            err.firstIdx = firstIdxRes;
            err.lastIdx = lastIdxRes;
            err.length = distancesRes[lastIdxRes] - distancesRes[firstIdxRes];
            err.errTransl = computeTranslationNorm(transformError) / lenSeg;
            err.errRot = computeRotationAngle(transformError) / lenSeg;
            errors.push_back(err);

            //			std::cout << "\n---\ninterval res [" <<
            // firstIdxRes << "," << lastIdxRes << "]  gt [" << firstIdxGt << "," <<
            // lastIdxGt << "]\n"
            //					<<
            //"transformsRes[firstIdxRes]\n" << transformsRes[firstIdxRes].matrix()
            //<< "\n" << "transformsRes[lastIdxRes]\n" <<
            // transformsRes[lastIdxRes].matrix() << "\n"
            //					<< "transformsGt[firstIdxGt]\n"
            //<< transformsGt[firstIdxGt].matrix() << "\n" <<
            //"transformsGt[lastIdxGt]\n" << transformsGt[lastIdxGt].matrix() <<
            //"\n"
            //					<< "transformDeltaRes\n" <<
            // transformDeltaRes.matrix() << "\n" << "transformDeltaGt\n" <<
            // transformDeltaGt.matrix() << "\n"
            //					<< "transformError\n" <<
            // transformError.matrix() << "\n" << "errTransl " << err.errTransl << "
            // errRot[deg] " << (180 / M_PI * err.errRot)
            //					<< std::endl;
            std::cout << "---\ninterval res [" << firstIdxRes << "," << lastIdxRes
                      << "](dist " << distancesRes[firstIdxRes] << ","
                      << distancesRes[lastIdxRes] << " m) "
                      << " factorRes " << factorRes << ", "
                      << "gt [" << firstIdxGt << "," << lastIdxGt << "](dist "
                      << distancesGt[firstIdxGt] << "," << distancesGt[lastIdxGt]
                      << " m) "
                      << " factorGt " << factorGt << ": "
                      << "errTransl[%] " << (100.0 * err.errTransl)
                      << "  errRot[10^{-1} * deg/m] "
                      << (18000.0 / M_PI * err.errRot) << std::endl;
            if (err.errTransl > 0.50)
            {
                std::cout << "transformDeltaRes\n"
                          << transformDeltaRes.matrix() << "\n"
                          << "transformDeltaGt\n"
                          << transformDeltaGt.matrix() << "\n"
                          << "transformError\n"
                          << transformError.matrix() << "\n";
            }

            // firstIdxRes = lastIdxRes;
            // firstIdxGt = lastIdxGt;
            // std::cout << "BEFORE: firstIdxRes " << firstIdxRes << ", firstIdxGt "
            // << firstIdxGt << "\n";
            firstIdxRes =
                findDistanceIncrIdx(distancesRes, firstIdxRes, lenStep, factorRes);
            firstIdxGt =
                findDistanceIncrIdx(distancesGt, firstIdxGt, lenStep, factorGt);
            // std::cout << " AFTER: firstIdxRes " << firstIdxRes << ", firstIdxGt "
            // << firstIdxGt << "\n";
        }
    }

    void computeErrorsInterp(const VectorTransform3 &transformsRes,
                             const std::vector<double> &distancesRes,
                             const VectorTransform3 &transformsGt,
                             const std::vector<double> &distancesGt,
                             double lenSeg,
                             double lenStep,
                             VectorErrorData &errors)
    {
        ErrorData err;
        Transform3 transformFirstRes, transformFirstGt, transformLastRes,
            transformLastGt;
        Transform3 transformDeltaRes, transformDeltaGt, transformError;
        int numRes, numGt, firstIdxGt, lastIdxGt, firstIdxRes, lastIdxRes;
        double distStart, factorFirstRes, factorFirstGt, factorLastRes, factorLastGt;
        // float len;

        if (transformsRes.size() != distancesRes.size() ||
            transformsGt.size() != distancesGt.size())
        {
            std::cerr << "Error: different size of distances and trasforms"
                      << std::endl;
            return;
        }
        numRes = transformsRes.size();
        numGt = transformsGt.size();
        if (numRes == 0 || numGt == 0)
        {
            std::cerr << "Error: one empty sequence: numRes " << numRes << " numGt "
                      << numGt << std::endl;
            return;
        }
        // std::cout << "numRes " << numRes << " numGt " << numGt << std::endl;

        // Compares the relative pose of path segments of length lenSeg in result
        // and groundtruth paths. The first and last pose correspond respectively to
        // distances distStart and distStart + lenSeg. The initial value of
        // distStart is 0 and is incremented of lenStep at each iteration
        distStart = 0.0;
        while (distStart + lenSeg < distancesRes.back() &&
               distStart + lenSeg < distancesGt.back())
        {
            // Computes the indices corresponding to distStart in result and
            // groundtruth paths
            firstIdxRes = findDistanceIdx(distancesRes, distStart, factorFirstRes);
            // std::cout << "firstIdxRes " << firstIdxRes << std::endl;
            firstIdxGt = findDistanceIdx(distancesGt, distStart, factorFirstGt);
            // std::cout << "factorFirstGt " << factorFirstGt << std::endl;
            if (firstIdxRes >= numRes || firstIdxGt >= numGt)
            {
                break;
            }
            // Computes the indices corresponding to distStart in result and
            // groundtruth paths
            lastIdxRes =
                findDistanceIdx(distancesRes, distStart + lenSeg, factorLastRes);
            // std::cout << "lastIdxRes " << lastIdxRes << std::endl;
            lastIdxGt =
                findDistanceIdx(distancesGt, distStart + lenSeg, factorLastGt);
            // std::cout << "lastIdxGt " << lastIdxGt << std::endl;
            if (lastIdxRes >= numRes || lastIdxGt >= numGt)
            {
                break;
            }
            // The first poses of result and groundtruth path segments are obtained
            // with interpolation
            if (firstIdxRes == 0)
            {
                transformFirstRes = transformsRes[0];
            }
            else
            {
                interpolateTransform(transformsRes[firstIdxRes - 1],
                                     transformsRes[firstIdxRes], factorFirstRes,
                                     transformFirstRes);
            }
            if (firstIdxGt == 0)
            {
                transformFirstGt = transformsGt[0];
            }
            else
            {
                interpolateTransform(transformsGt[firstIdxGt - 1],
                                     transformsGt[firstIdxGt], factorFirstGt,
                                     transformFirstGt);
            }
            // The last poses of result and groundtruth path segments are obtained
            // with interpolation
            interpolateTransform(transformsRes[lastIdxRes - 1],
                                 transformsRes[lastIdxRes], factorLastRes,
                                 transformLastRes);
            interpolateTransform(transformsGt[lastIdxGt - 1],
                                 transformsGt[lastIdxGt], factorLastGt,
                                 transformLastGt);
            // Computes the relative pose of result and groundtruth; then the error
            transformDeltaRes = transformFirstRes.inverse() * transformLastRes;
            transformDeltaGt = transformFirstGt.inverse() * transformLastGt;
            transformError = transformDeltaGt.inverse() * transformDeltaRes;
            // Saves the error
            err.firstIdx = firstIdxRes;
            err.lastIdx = lastIdxRes;
            err.length = distancesRes[lastIdxRes] - distancesRes[firstIdxRes];
            err.errTransl = computeTranslationNorm(transformError) / lenSeg;
            err.errRot = computeRotationAngle(transformError) / lenSeg;
            errors.push_back(err);

            std::cout << "res [" << firstIdxRes << "," << lastIdxRes << "](dist "
                      << (distancesRes[firstIdxRes] * (1.0 - factorFirstRes) +
                          distancesRes[firstIdxRes + 1] * factorFirstRes)
                      << ","
                      << (distancesRes[lastIdxRes] * (1.0 - factorLastRes) +
                          distancesRes[lastIdxRes + 1] * factorLastRes)
                      << ") "
                      << "gt [" << firstIdxGt << "," << lastIdxGt << "](dist "
                      << (distancesGt[firstIdxGt] * (1.0 - factorFirstGt) +
                          distancesGt[firstIdxGt + 1] * factorFirstGt)
                      << ","
                      << (distancesGt[lastIdxGt] * (1.0 - factorLastGt) +
                          distancesGt[lastIdxGt + 1] * factorLastGt)
                      << "): "
                      << "errTransl[%] " << (100.0 * err.errTransl)
                      << "  errRot[10^{-1} * deg/m] "
                      << (18000.0 / M_PI * err.errRot) << std::endl;

            // Increments the starting distance
            distStart += lenStep;
        }
    }

    int readGraph(const std::string &filename,
                  std::vector<Node> &nodes,
                  VectorTransform2 &gts,
                  VectorTransform2 &odoms,
                  std::vector<Edge> &edges)
    {
        std::string line, comment, label;
        scan_overlap::Vector2 p;
        size_t pos;
        int count;

        std::ifstream file(filename);
        if (!file)
        {
            std::cerr << "Cannot open file \"" << filename << "\"" << std::endl;
            return 0;
        }

        nodes.clear();
        gts.clear();
        odoms.clear();
        edges.clear();

        count = 0;
        while (!file.eof())
        {
            std::getline(file, line);
            // Remove comments starting with '#'
            comment = "";
            pos = line.find_first_of('#');
            if (pos != std::string::npos)
            {
                if (pos == 0)
                    continue;
                comment = line.substr(pos + 1, line.size());
                line = line.substr(0, pos);
            }
            // Parse the line (after comment removal)
            std::stringstream ssl(line);
            if (line.find("EDGE") == 0)
            {
                int src, dst;
                ssl >> label;
                if (ssl >> src >> dst)
                    edges.push_back(scan_overlap::Edge(src, dst));
            }
            else if (line.find("NODE") == 0)
            {
                double pX, pY, pT, oX, oY, oT;
                int id;
                scan_overlap::Vector2 p;
                scan_overlap::Cloud cloud;
                ssl >> label;
                if (ssl >> id >> pX >> pY >> pT >> oX >> oY >> oT)
                {
                    scan_overlap::Transform2 odom, gt;
                    odom.linear() = Eigen::Rotation2Dd(oT).matrix();
                    odom.translation() = scan_overlap::Vector2(oX, oY);

                    gt.linear() = Eigen::Rotation2Dd(pT).matrix();
                    gt.translation() = scan_overlap::Vector2(pX, pY);

                    odoms.push_back(odom);
                    gts.push_back(gt);
                }
                else
                    continue;

                while (ssl >> p.x() >> p.y())
                    cloud.push_back(p);
                nodes.push_back(Node(id, cloud));
            }
        }
        file.close();

        return count;
    }

    void saveGraph(const scan_overlap::Graph &graph,
                   const std::vector<scan_overlap::Cloud> &clouds,
                   const scan_overlap::VectorTransform2 &gts,
                   const scan_overlap::VectorTransform2 &odoms,
                   const std::string &fileName)
    {
        std::ofstream f(fileName);
        f << "# format\n"
          << "# NODE id poseX poseY poseTheta odomX odomY odomTheta x0 y0 ... xN yN\n"
          << "# EDGE idSrc idDst\n";
        for (int i = 0; i < graph.size(); i++)
        {
            scan_overlap::Node n = graph[i];
            scan_overlap::Cloud cloud = clouds[n.id];

            scan_overlap::Transform2 gt = gts[i];
            Eigen::Translation2d gtT = Eigen::Translation2d(gt.translation());
            double gtA = Eigen::Rotation2Dd(gt.linear()).angle();

            scan_overlap::Transform2 odom = odoms[i];
            Eigen::Translation2d odomT = Eigen::Translation2d(odom.translation());
            double odomA = Eigen::Rotation2Dd(odom.linear()).angle();

            f << "NODE " << n.id << " " << gtT.x() << " " << gtT.y() << " " << gtA << " " << odomT.x() << " " << odomT.y() << " " << odomA << " ";

            for (const auto &p : cloud)
                f << p.x() << " " << p.y() << " ";

            f << "\n";

            for (int a : n.adj)
                f << "EDGE " << a << " " << n.id << "\n";
        }
    }

} // namespace  scan_overlap
