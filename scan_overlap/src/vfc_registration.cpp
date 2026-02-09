#include "vfc_registration.h"

//#include "libvfc/CurveMatching/CurveSignature.h"

namespace scan_overlap {

    cv::Point2f toPointCv(const Vector2& peig) {
        cv::Point2f pcv;
        pcv.x = peig.x();
        pcv.y = peig.y();
        return pcv;
    }

    VFCRegistration::VFCRegistration() {
    }

    VFCRegistration::~VFCRegistration() {
    }

    void VFCRegistration::setPointSetSrc(const VectorVector2& points) {
        pointsSrc_ = points;
    }

    void VFCRegistration::setPointSetDst(const VectorVector2& points) {
        pointsDst_ = points;
    }

    // double VFCRegistration::computeRigidTransform(Transform2& transf) {
    //     std::vector<cv::Point2f> contourCvSrc;
    //     std::vector<cv::Point2f> contourCvDst;
    //     std::vector<cv::Point2f> pointsAssocSrc;
    //     std::vector<cv::Point2f> pointsAssocDst;
    //     std::vector<std::pair<int, int> > correspondences;
    //     int k = 1;
    //     double rotSD;

    //     // Converts Eigen points to VFC
    //     //    convertEigenToCvPoints(contourSrc_, contourCvSrc);
    //     //    convertEigenToCvPoints(contourDst_, contourCvDst);
    //     //
    //     //    int lenSrc, offsetSrc, lenDst, offsetDst;
    //     //    double db_compare_score;
    //     //    CompareCurvesUsingSignatureDB(contourCvSrc, contourCvDst, lenSrc, offsetSrc, lenDst, offsetDst, db_compare_score);
    //     //
    //     //    std::vector<cv::Point2f> pointsAssocSrc(contourCvSrc.begin() + offsetSrc, contourCvSrc.begin() + offsetSrc + lenSrc);
    //     //    std::vector<cv::Point2f> pointsAssocDst(contourCvDst.begin() + offsetDst, contourCvDst.begin() + offsetDst + lenDst);
    //     PCARegistration pcaReg;
    //     Transform2 transfPca;
    //     pcaReg.setPointSetSrc(pointsSrc_);
    //     pcaReg.setPointSetDst(pointsDst_);
    //     pcaReg.computeRigidTransform(transfPca);

    //     PointReaderWriter prwDst(pointsDst_);
    //     std::vector<int> indicesNearest;
    //     std::vector<double> distancesNearest;
    //     Eigen::Vector2d psrc;
    //     for (auto& p : pointsSrc_) {
    //         psrc = transfPca * p;
    //         prwDst.findKNearest(psrc, k, indicesNearest, distancesNearest);
    //         //std::cout << "source p " << p.transpose() << " indicesNearest.size() " << indicesNearest.size()  << " distancesNearest.size() " << distancesNearest.size() << std::endl;
    //         for (auto& idx : indicesNearest) {
    //             cv::Point2f p1 = toPointCv(p);
    //             cv::Point2f p2 = toPointCv(pointsDst_[idx]);
    //             pointsAssocSrc.push_back(p1);
    //             pointsAssocDst.push_back(p2);
    //         }
    //     }
    //     //std::cout << "associations: pointsAssocSrc.size() " << pointsAssocSrc.size() << ", pointsAssocDst.size() " << pointsAssocDst.size() << std::endl;


    //     //std::cout << "  putative correspondences src " << pointsAssocSrc.size() << " to dst " << pointsAssocDst.size() << std::endl;
    //     // Inserts all the potential correspondences as candidates
    //     //    int num = pointsCvSrc.size() * pointsCvDst.size();
    //     //    std::cout << __FILE__ << "," << __LINE__ << ": total associations " << num << std::endl;
    //     //    pointsAssocSrc.reserve(num);
    //     //    pointsAssocDst.reserve(num);
    //     //    for (auto& psrc : pointsCvSrc) {
    //     //        for (auto& pdst : pointsCvDst) {
    //     //            pointsAssocSrc.push_back(psrc);
    //     //            pointsAssocDst.push_back(pdst);
    //     //        }
    //     //    }

    //     // Calls the VFC class to removes outlier correspondences 
    //     VFC myvfc;
    //     myvfc.setData(pointsAssocSrc, pointsAssocDst);
    //     myvfc.optimize();
    //     std::vector<int> matchIdx = myvfc.obtainCorrectMatch();

    //     //std::cout << __FILE__ << "," << __LINE__ << ": total correct associations " << matchIdx.size() << std::endl;
    //     for (int i = 0; i < matchIdx.size(); ++i) {
    //         correspondences.push_back(std::make_pair(i, matchIdx[i]));
    //         //std::cout << " matching " << i << ", " << matchIdx[i] << std::endl;
    //     }
    //     //std::cout << __FILE__ << "," << __LINE__ << ": correspondences.size() " << correspondences.size() << std::endl;

    //     computeTransform(pointsSrc_, pointsDst_, correspondences, transf, rotSD);
    //     transf = transf.inverse();
    //     rotSD = -rotSD;
    //     //std::cout << __FILE__ << "," << __LINE__ << ": rotSD[deg] " << (180.0/M_PI*rotSD) << ", transf\n" << transf.matrix() << std::endl;

    //     // Computes
    //     //transf = Trasform2::Identity();
    //     //  transf.prerotate(rotSD);
    //     //  transf.pretranslate(transl); 
    //     return rotSD;
    // }

    double VFCRegistration::computeRigidTransform(Transform2& transf, Transform2& guess) {
        std::vector<cv::Point2f> contourCvSrc;
        std::vector<cv::Point2f> contourCvDst;
        std::vector<cv::Point2f> pointsAssocSrc;
        std::vector<cv::Point2f> pointsAssocDst;
        std::vector<std::pair<int, int> > correspondences;
        int k = 3;
        double rotSD;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudDst(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < pointsDst_.size(); ++i) {
            pclCloudDst->points.push_back(pcl::PointXYZ(pointsDst_[i].x(), pointsDst_[i].y(), 0.0));
        }   

        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudSrcTransf(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < pointsSrc_.size(); ++i) {
            Vector2 ptTransf =  guess * pointsSrc_[i];
            pclCloudSrcTransf->points.push_back(pcl::PointXYZ(ptTransf.x(), ptTransf.y(), 0.0));
        }   


        kdtree.setInputCloud (pclCloudDst);

        for (int i = 0; i < pclCloudSrcTransf->size(); ++i) {
            std::vector<int> pointIdxKNNSearch(k);
            std::vector<float> pointKNNSquaredDistance(k);
            auto ptTransf = pclCloudSrcTransf->at(i);
            if ( kdtree.nearestKSearch(ptTransf, k, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
            {
                for (int j = 0; j < pointIdxKNNSearch.size(); ++j)
                {
                    pointsAssocSrc.push_back(toPointCv(pointsSrc_[i]));
                    pointsAssocDst.push_back(toPointCv(pointsDst_[pointIdxKNNSearch[j]]));
                }
            }
        }

        // Calls the VFC class to removes outlier correspondences 
        VFC myvfc;
        myvfc.setData(pointsAssocSrc, pointsAssocDst);
        myvfc.optimize();
        std::vector<int> matchIdx = myvfc.obtainCorrectMatch();

        //std::cout << __FILE__ << "," << __LINE__ << ": total correct associations " << matchIdx.size() << std::endl;
        for (int i = 0; i < matchIdx.size(); ++i) {
            correspondences.push_back(std::make_pair(i, matchIdx[i]));
            //std::cout << " matching " << i << ", " << matchIdx[i] << std::endl;
        }
        //std::cout << __FILE__ << "," << __LINE__ << ": correspondences.size() " << correspondences.size() << std::endl;

        computeTransform(pointsSrc_, pointsDst_, correspondences, transf, rotSD);
        transf = transf.inverse();
        rotSD = -rotSD;
        //std::cout << __FILE__ << "," << __LINE__ << ": rotSD[deg] " << (180.0/M_PI*rotSD) << ", transf\n" << transf.matrix() << std::endl;

        // Computes
        //transf = Trasform2::Identity();
        //  transf.prerotate(rotSD);
        //  transf.pretranslate(transl); 
        return rotSD;
    }

    int VFCRegistration::computeTransform(const VectorVector2& pointsSrc, const VectorVector2& pointsDst, const std::vector<std::pair<int, int> >& associations, Transform2& transform, double& theta) {
        Eigen::Vector2d tsrc = Eigen::Vector2d::Zero();
        Eigen::Vector2d tdst = Eigen::Vector2d::Zero();
        Eigen::Matrix2d S = Eigen::Matrix2d::Zero();
        int n = 0;

        // Computes mean values on each point set for valid associations
        for (auto& a : associations) {
            if (0 <= a.first && a.first < pointsSrc.size() && 0 <= a.second && a.second < pointsDst.size()) {
                tsrc += pointsSrc[a.first];
                tdst += pointsDst[a.second];
                n++;
            }
        }

        // Checks minimum number of valid points
        if (n < 2) {
            std::cerr << __FILE__ << "," << __LINE__ << ": not enough associations between the two point sets! only " << n << " < 2" << std::endl;
            return n;
        }

        // Computes transformation
        tsrc = (1.0 / n) * tsrc;
        tdst = (1.0 / n) * tdst;
        for (auto& a : associations) {
            if (0 <= a.first && a.first < pointsSrc.size() && 0 <= a.second && a.second < pointsDst.size()) {
                S += (pointsDst[a.second] - tdst) * (pointsSrc[a.first] - tsrc).transpose();
            }
        }
        theta = atan2(S(0, 1) - S(1, 0), S(0, 0) + S(1, 1));
        Eigen::Rotation2Dd rot(theta);
        //std::cout << "convert rot toRotationMatrix()\n" << rot.toRotationMatrix() << std::endl;
        Eigen::Vector2d transl = tsrc - (rot * tdst);
        transform = Eigen::Affine2d::Identity();
        transform = rot;
        transform.pretranslate(transl);

        return n;
    }

    void VFCRegistration::convertEigenToCvKeypoints(const VectorVector2& points, std::vector<cv::KeyPoint>& keypoints) {
        keypoints.resize(points.size());
        for (int i = 0; i < points.size(); ++i) {
            keypoints[i].pt.x = points[i].x();
            keypoints[i].pt.y = points[i].y();
        }
    }

    void VFCRegistration::convertEigenToCvPoints(const VectorVector2& pointsEigen, std::vector<cv::Point2f>& pointsCv) {
        pointsCv.resize(pointsEigen.size());
        for (int i = 0; i < pointsEigen.size(); ++i) {
            pointsCv[i].x = pointsEigen[i].x();
            pointsCv[i].y = pointsEigen[i].y();
        }
    }

}
