#ifndef SCAN_OVERLAP_SIMPLE_ICP_REGISTRATION_H_
#define SCAN_OVERLAP_SIMPLE_ICP_REGISTRATION_H_

#include <iostream>
#include <vector>

#include "types.h"

namespace scan_overlap {

    class SimpleIcpRegistration {
    public:
        /** Constructor.
         */
        SimpleIcpRegistration(int sizeMin = 0);

        /** Destructor.
         */
        ~SimpleIcpRegistration();

        /** Enables the removal of centroids from each point set. 
         */
        void enableRemoveCentroid(bool rc) {
            removeCentroid_ = rc;
        }

        /** Sets the source point set.
         */
        void setPointSetSrc(const VectorVector2& points);

        /** Sets the destination point set.
         */
        void setPointSetDst(const VectorVector2& points);

        /** Computes the transformation between source and destination. 
         */
        double computeRigidTransform(Transform2& transf);

        /** Computes the transformation between source and destination. 
         */
        double computeRigidTransform(Transform2& transf, const Transform2& guess);

        /** Computes the transformation between source and destination. 
         */
        double computeRigidTransform(Transform2& transf, double tx, double ty, double theta);

    private:
        double* valSrc_;
        double* valDst_;
        int sizeSrc_;
        int sizeDst_;
        Vector2 centroidSrc_;
        Vector2 centroidDst_;
        bool removeCentroid_;

        void assignArray(const VectorVector2& points, double* val, int& size, Vector2& centroid);
    };

}

#endif //SCAN_OVERLAP_SIMPLE_ICP_REGISTRATION_H_
