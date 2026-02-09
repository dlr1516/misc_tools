#ifndef VFC_REGISTRATION_H
#define VFC_REGISTRATION_H

//#include <Eigen/Dense>
#include <Eigen/Eigenvalues> 
#include <vector>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "thirdparty/libvfc/vfc.h"

#include <iostream>

#include <opencv2/core/core.hpp>

#include "types.h"

namespace scan_overlap {

    /** Class that applies Enhanced Correlation Coefficient (ECC), an image registration 
     * method. 
     */
    class VFCRegistration {
    public:
        /** Default constructor. 
         */
        VFCRegistration();

        /** Default denstructor. 
         */
        virtual ~VFCRegistration();

        /** Sets the source point set and converts it to an image.
         */
        void setPointSetSrc(const VectorVector2& points);

        /** Sets the source point set and converts it to an image.
         */
        void setPointSetDst(const VectorVector2& points);

        /** Computes the transformation between source and destination, computing initguess internally using PCA.
         * Returns rotation angle.
         */
        double computeRigidTransform(Transform2& transf);

        /**
         * Computes the transformation between source and destination, using the provided guess as initial transformation.
         * Returns rotation angle.
         */
        double computeRigidTransform(Transform2& transf, Transform2& guess);

        int computeTransform(const VectorVector2& pointsSrc, const VectorVector2& pointsDst, const std::vector<std::pair<int, int> >& associations, Transform2& transform, double& theta);


    private:
        VectorVector2 pointsSrc_;
        VectorVector2 pointsDst_;
        VectorVector2 contourSrc_;
        VectorVector2 contourDst_;

        void convertEigenToCvKeypoints(const VectorVector2& points, std::vector<cv::KeyPoint>& keypoints);

        void convertEigenToCvPoints(const VectorVector2& pointsEigen, std::vector<cv::Point2f>& pointsCv);
    };

}

#endif //VFC_REGISTRATION_H
