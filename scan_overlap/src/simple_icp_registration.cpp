#include "simple_icp_registration.h"
#include "thirdparty/libicp/icpPointToPlane.h"
#include "thirdparty/libicp/icpPointToPoint.h"

namespace scan_overlap {

    SimpleIcpRegistration::SimpleIcpRegistration(int sizeMin)
    : removeCentroid_(false) {
        //  std::cout << __PRETTY_FUNCTION__ << std::endl;
        valSrc_ = new double[2 * sizeMin];
        valDst_ = new double[2 * sizeMin];
        sizeSrc_ = sizeMin;
        sizeDst_ = sizeMin;
        //  std::cout << __PRETTY_FUNCTION__ << ": done" << std::endl;
    }

    SimpleIcpRegistration::~SimpleIcpRegistration() {
        //  std::cout << __FUNCTION__ << std::endl;
        delete valSrc_;
        delete valDst_;
    }

    void SimpleIcpRegistration::setPointSetSrc(const VectorVector2& points) {
        assignArray(points, valSrc_, sizeSrc_, centroidSrc_);
    }

    void SimpleIcpRegistration::setPointSetDst(const VectorVector2& points) {
        assignArray(points, valDst_, sizeDst_, centroidDst_);
    }

    double SimpleIcpRegistration::computeRigidTransform(Transform2& transf) {
        return computeRigidTransform(transf, 0.0, 0.0, 0.0);
    }

    double SimpleIcpRegistration::computeRigidTransform(Transform2& transf, const Transform2& guess) {
        Eigen::Matrix3d matr = guess.matrix();
        double tx = matr(0, 2);
        double ty = matr(1, 2);
        double theta = atan2(matr(1, 0), matr(0, 0));
        return computeRigidTransform(transf, tx, ty, theta);
    }

    double SimpleIcpRegistration::computeRigidTransform(Transform2& transf, double tx, double ty, double theta) {
        Matrix R = Matrix::eye(2);
        Matrix t(2, 1);
        float cost = cos(theta);
        float sint = sin(theta);
        double angle;

        R.val[0][0] = R.val[1][1] = cost;
        R.val[0][1] = -sint;
        R.val[1][0] = sint;
        t.val[0][0] = tx;
        t.val[1][0] = ty;
        //  std::cout << "\n" << __FUNCTION__ << ": Initial guess: rotation\n" << R << "\ntranslation\n" << t << "\nangle[deg] " << (180.0/M_PI*theta)<< std::endl;

        IcpPointToPoint icp(valSrc_, sizeSrc_, 2);
        icp.setMaxIterations(200);
        icp.fit(valDst_, sizeDst_, R, t, -1);

        angle = atan2(R.val[1][0], R.val[0][0]);
        transf = Transform2::Identity();
        transf.prerotate(angle);
        transf.pretranslate(Vector2(t.val[0][0], t.val[1][0]));

        //  std::cout << "\n" << __FUNCTION__ << ": Final: rotation\n" << R << "\ntranslation\n" << t << "\nangle[deg] " << (180.0/M_PI*angle) << std::endl;
        return angle;
    }

    void SimpleIcpRegistration::assignArray(const VectorVector2& points, double* val, int& size, Vector2& centroid) {
        centroid = Vector2::Zero();
        // If the array size is not large enough, a new allocation is required
        if (points.size() > 2 * size) {
            delete val;
            val = new double[2 * points.size()];
        }
        // Centroid removal (if enabled)
        if (removeCentroid_) {
            for (int i = 0; i < points.size(); ++i) {
                centroid += points[i];
            }
            if (points.size() > 0) {
                centroid = centroid / points.size();
            }
        }
        // Copies the values
        for (int i = 0; i < points.size(); ++i) {
            val[2 * i] = points[i].x() - centroid.x();
            val[2 * i + 1] = points[i].y() - centroid.y();
        }
        size = points.size();
    }

}