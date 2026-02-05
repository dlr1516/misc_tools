#include <iostream>

#include "../src/thirdparty/libicp/icpPointToPlane.h"

#include "../src/thirdparty/libvfc/vfc.h"
// #include "../src/thirdparty/libvfc/featureMatch.h"

#include <opencv2/core/core.hpp>


int main (int argc, char** argv) {

    /** 
     * ICP
     */

    // define a 3 dim problem with 10000 model points
    // and 10000 template points:
    int32_t dim = 3;
    int32_t num = 10000;

    // allocate model and template memory
    double* M = (double*) calloc(3 * num, sizeof (double));
    double* T = (double*) calloc(3 * num, sizeof (double));

    // set model and template points
    std::cout << endl << "Creating model with 10000 points ..." << endl;
    cout << "Creating template by shifting model by (1,1,1) ..." << endl;
    int32_t k = 0;
    for (double x = -2; x < 2; x += 0.04) {
        for (double y = -2; y < 2; y += 0.04) {
            double z = 5 * x * exp(-x * x - y * y);
            M[k * 3 + 0] = x;
            M[k * 3 + 1] = y;
            M[k * 3 + 2] = z;
            T[k * 3 + 0] = x - 1;
            T[k * 3 + 1] = y - 1;
            T[k * 3 + 2] = z - 1;
            k++;
        }
    }

    // start with identity as initial transformation
    // in practice you might want to use some kind of prediction here
    Matrix R = Matrix::eye(3);
    Matrix t(3, 1);

    // run point-to-plane ICP (-1 = no outlier threshold)
    cout << endl << "Running ICP (point-to-plane, no outliers)" << endl;
    IcpPointToPlane icp(M, num, dim);
    icp.fit(T, num, R, t, -1);

    // results
    cout << endl << "Transformation results:" << endl;
    cout << "R:" << endl << R << endl << endl;
    cout << "t:" << endl << t << endl << endl;

    // free memory
    free(M);
    free(T);

    /** 
     * VFC
     */

    cv::Mat img_1, img_2;
    int vfcMethod = 1; // 0: SURF; 1: ORB

     if (!img_1.data || !img_2.data
            || img_1.rows != img_2.rows || img_1.cols != img_2.cols) {
        return -1;
    }

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    std::vector< DMatch > matches, correctMatches;

    // initial matching by SURF or ORB
    // if (vfcMethod == 0) {
    //     surfInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
    //             descriptors_1, descriptors_2, matches);
    // } else {
    //     orbInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
    //             descriptors_1, descriptors_2, matches);
    // }

    // //  Remove mismatches by vector field consensus (VFC)
    // vfcMatch(keypoints_1, keypoints_2, matches, correctMatches);

    // // visualization
    // visualizeMatchingResults(img_1, img_2, keypoints_1, keypoints_2,
    //         matches, correctMatches);



    return 0;
}