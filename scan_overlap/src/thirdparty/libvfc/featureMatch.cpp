#include "featureMatch.h"
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
//     Regularized Vector Field Learning with Sparse Approximation for Mismatch Removal, 
//     Pattern Recognition, 46(12), pp. 3519-3532, 2013.

void surfInitMatchImagePair(Mat &img_1, Mat &img_2,
        std::vector<KeyPoint> &keypoints_1, std::vector<KeyPoint> &keypoints_2,
        Mat &descriptors_1, Mat &descriptors_2,
        std::vector<DMatch> &matches) {
    double t = (double) getTickCount();
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    cv::SurfFeatureDetector detector(minHessian);
    detector.detect(img_1, keypoints_1);
    detector.detect(img_2, keypoints_2);
    if (keypoints_1.size() < 3 || keypoints_2.size() < 3)
        return;

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    extractor.compute(img_1, keypoints_1, descriptors_1);
    extractor.compute(img_2, keypoints_2, descriptors_2);
    t = 1000 * ((double) getTickCount() - t) / getTickFrequency();
    cout << "feature extraction time (ms): " << t << endl;

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    t = (double) getTickCount();
    BFMatcher matcher(NORM_L2);
    matcher.match(descriptors_1, descriptors_2, matches);
    t = 1000 * ((double) getTickCount() - t) / getTickFrequency();
    cout << "initial matching time (ms): " << t << endl;
}

void orbInitMatchImagePair(Mat &img_1, Mat &img_2,
        std::vector<KeyPoint> &keypoints_1, std::vector<KeyPoint> &keypoints_2,
        Mat &descriptors_1, Mat &descriptors_2,
        std::vector<DMatch> &matches) {
    double t = (double) getTickCount();
    //-- Step 1: Detect the keypoints using ORB Detector
    cv::ORB orb;
    ;
    orb.detect(img_1, keypoints_1);
    orb.detect(img_2, keypoints_2);
    if (keypoints_1.size() < 3 || keypoints_2.size() < 3)
        return;

    //-- Step 2: Calculate descriptors (feature vectors)
    orb.compute(img_1, keypoints_1, descriptors_1);
    orb.compute(img_2, keypoints_2, descriptors_2);
    t = 1000 * ((double) getTickCount() - t) / getTickFrequency();
    cout << "feature extraction time (ms): " << t << endl;

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    t = (double) getTickCount();
    BFMatcher matcher(NORM_HAMMING, true);
    matcher.match(descriptors_1, descriptors_2, matches);
    t = 1000 * ((double) getTickCount() - t) / getTickFrequency();
    cout << "initial matching time (ms): " << t << endl;
}

void vfcMatch(std::vector<cv::KeyPoint> &mvKeys1, std::vector<cv::KeyPoint> &mvKeys2,
        std::vector<DMatch> &matches, std::vector<DMatch> &correctMatches) {
    double t = (double) getTickCount();
    // preprocess data format
    vector<Point2f> X;
    vector<Point2f> Y;
    for (unsigned int i = 0; i < matches.size(); i++) {
        int idx1 = matches[i].queryIdx;
        int idx2 = matches[i].trainIdx;
        X.push_back(mvKeys1[idx1].pt);
        Y.push_back(mvKeys2[idx2].pt);
    }
    // main process
    VFC myvfc;
    myvfc.setData(X, Y);
    myvfc.optimize();
    vector<int> matchIdx = myvfc.obtainCorrectMatch();

    // postprocess data format
    correctMatches.clear();
    for (unsigned int i = 0; i < matchIdx.size(); i++) {
        int idx = matchIdx[i];
        correctMatches.push_back(matches[idx]);
    }
    t = 1000 * ((double) getTickCount() - t) / getTickFrequency();
    cout << "VFC time (ms): " << t << endl;
}

void visualizeMatchingResults(Mat &img_1, Mat &img_2,
        std::vector<KeyPoint> &keypoints_1, std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches, std::vector< DMatch > correctMatches) {
    int h = img_1.rows;
    int w = img_1.cols;

    // visualization
    //-- Draw matches
    Mat img_matches;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_matches);
    imshow("Matches", img_matches);
    //-- Draw mismatch removal result
    Mat img_correctMatches;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, correctMatches, img_correctMatches);
    imshow("Detected Correct Matches", img_correctMatches);

    // visualize vector field
    visualizeVectorField(keypoints_1, keypoints_2, matches, h, w, "original matches");
    visualizeVectorField(keypoints_1, keypoints_2, correctMatches, h, w, "VFC matches");

    //-- write image
    //imwrite("C:\\jizhao\\intial_match.png", img_matches);
    //imwrite("C:\\jizhao\\VFC_result.png", img_correctMatches);
}

void visualizeVectorField(std::vector<KeyPoint> &keypoints_1,
        std::vector<KeyPoint> &keypoints_2,
        std::vector<DMatch> &matches, int h, int w, string winName) {
    Mat img = cv::Mat::zeros(cv::Size(w, h), CV_8UC3);
    img = Scalar(255, 255, 255);

    double arrowLen = 10.0;
    double diffAngle = MY_PI / 6;
    double angle;
    for (int i = 0; i < matches.size(); i++) {
        int idx1 = matches[i].queryIdx;
        int idx2 = matches[i].trainIdx;
        Point2f pHead = keypoints_1[idx1].pt;
        Point2f pTail = keypoints_2[idx2].pt;
        angle = atan2((double) pTail.y - pHead.y, (double) pTail.x - pHead.x);
        plotArrow(img, pHead, pTail, arrowLen, diffAngle);
    }
    imshow(winName, img);
    waitKey(1);
}

void plotArrow(Mat &img, Point2f pHead, Point2f pTail, double arrowLen, double diffAngle) {
    cv::Point p;
    double angle = atan2((double) pTail.y - pHead.y, (double) pTail.x - pHead.x);

    cv::line(img, pTail, pHead, cv::CV_RGB(0, 255, 0), 2, 8);

    p.x = (int) (pHead.x + arrowLen * cos(angle + diffAngle));
    p.y = (int) (pHead.y + arrowLen * sin(angle + diffAngle));
    cv::line(img, p, pHead, cv::CV_RGB(0, 255, 0), 1, cv::CV_AA, 0);

    p.x = (int) (pHead.x + arrowLen * cos(angle - diffAngle));
    p.y = (int) (pHead.y + arrowLen * sin(angle - diffAngle));
    line(img, p, pHead, cv::CV_RGB(0, 255, 0), 1, cv::CV_AA, 0);
}
