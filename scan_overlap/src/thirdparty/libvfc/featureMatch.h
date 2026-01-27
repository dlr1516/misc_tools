#ifndef _FEATURE_MATCH_H
#define _FEATURE_MATCH_H
// Mismatch removal by vector field consensus (VFC)
// Author: Ji Zhao
// Date:   01/25/2015
// Email:  zhaoji84@gmail.com
//
// Reference
// [1] Jiayi Ma, Ji Zhao, Jinwen Tian, Alan Yuille, and Zhuowen Tu.
//     Robust Point Matching via Vector Field Consensus,
//     IEEE Transactions on Image Processing, 23(4), pp. 1706-1721, 2014.
// [2] Jiayi Ma, Ji Zhao, Jinwen Tian, et al.
//     Regularized Vector Field Learning with Sparse Approximation for Mismatch
//     Removal, Pattern Recognition, 46(12), pp. 3519-3532, 2013.

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <vector>
#include "vfc.h"

#define MY_PI 3.1415926

using namespace cv;

void surfInitMatchImagePair(Mat& img_1,
                            Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            Mat& descriptors_1,
                            Mat& descriptors_2,
                            std::vector<DMatch>& matches);

void orbInitMatchImagePair(Mat& img_1,
                           Mat& img_2,
                           std::vector<KeyPoint>& keypoints_1,
                           std::vector<KeyPoint>& keypoints_2,
                           Mat& descriptors_1,
                           Mat& descriptors_2,
                           std::vector<DMatch>& matches);

void vfcMatch(std::vector<cv::KeyPoint>& mvKeys1,
              std::vector<cv::KeyPoint>& mvKeys2,
              std::vector<DMatch>& matches,
              std::vector<DMatch>& correctMatches);

void visualizeMatchingResults(Mat& img_1,
                              Mat& img_2,
                              std::vector<KeyPoint>& keypoints_1,
                              std::vector<KeyPoint>& keypoints_2,
                              std::vector<DMatch>& matches,
                              std::vector<DMatch> correctMatches);

void visualizeVectorField(std::vector<KeyPoint>& keypoints_1,
                          std::vector<KeyPoint>& keypoints_2,
                          std::vector<DMatch>& matches,
                          int h,
                          int w,
                          string winName);

void plotArrow(Mat& img,
               Point2f pHead,
               Point2f pTail,
               double spinSize,
               double diffAngle);

#endif