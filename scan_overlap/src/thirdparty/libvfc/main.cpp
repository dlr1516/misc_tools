// This is a demo for mismatch removal by vector field consensus (VFC)
//
// Reference
// [1] Jiayi Ma, Ji Zhao, Jinwen Tian, Alan Yuille, and Zhuowen Tu.
//     Robust Point Matching via Vector Field Consensus, 
//     IEEE Transactions on Image Processing, 23(4), pp. 1706-1721, 2014.
// [2] Jiayi Ma, Ji Zhao, Jinwen Tian, et al.
//     Regularized Vector Field Learning with Sparse Approximation for Mismatch Removal, 
//     Pattern Recognition, 46(12), pp. 3519-3532, 2013.

#include "opencv2/core/core.hpp"
#include "vfc.h"
#include "featureMatch.h"
using namespace cv;

int main() {
    string pathImg1 = "C://church1.jpg";
    string pathImg2 = "C://church2.jpg";
    int method = 1; // 0: SURF; 1: ORB
    Mat img_1 = imread(pathImg1, CV_LOAD_IMAGE_GRAYSCALE);
    Mat img_2 = imread(pathImg2, CV_LOAD_IMAGE_GRAYSCALE);

    if (!img_1.data || !img_2.data
            || img_1.rows != img_2.rows || img_1.cols != img_2.cols) {
        return -1;
    }

    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    std::vector< DMatch > matches, correctMatches;

    // initial matching by SURF or ORB
    if (method == 0) {
        surfInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
                descriptors_1, descriptors_2, matches);
    } else {
        orbInitMatchImagePair(img_1, img_2, keypoints_1, keypoints_2,
                descriptors_1, descriptors_2, matches);
    }

    //  Remove mismatches by vector field consensus (VFC)
    vfcMatch(keypoints_1, keypoints_2, matches, correctMatches);

    // visualization
    visualizeMatchingResults(img_1, img_2, keypoints_1, keypoints_2,
            matches, correctMatches);
    waitKey(0);

    return 0;
}



