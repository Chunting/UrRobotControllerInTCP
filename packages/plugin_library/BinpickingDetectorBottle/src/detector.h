//
// Created by vimjian on 17-4-14.
//

#ifndef OPENNI_GRABBER_DETECTOR_H
#define OPENNI_GRABBER_DETECTOR_H

#include "opencv2/opencv.hpp"
#include "colorDetector.h"
#include <vector>

using namespace cv;
using namespace std;

struct graspRect
        {

    double area;
    Point2d rectCenter;
};

class Detector
{
private:
    ColorDetector colorDet;
    double imageScale;
    vector<graspRect> candidateRects;
    int regionNum;
    Mat m_colorImageShow;
    Mat m_colorImage;
public:
    vector<Point2i> candidatePoints;
public:
    void FindGraspPoint(Mat &colorMat,Point& graspPoint);
    void FindGraspRegion_ColorLabel(Mat &colorMat);
    void FindGraspRegion_ColorLabelHSV(Mat &colorMat);
    void FindGraspRegion_barCode_connectRegion(Mat &colorMat);
    void FindGraspRegion_barCode_brightness(Mat &colorMat);
    void FindGraspRegion(Mat &colorMat ,Mat &depthMat);
    void FindGraspRegion_combine_label_barCode(Mat &colorMat);
    void SetColorImage(Mat& colorMat);
    Detector();

private:
    void drawHist(Mat &sourceMat, int binsNum, float minRange, float maxRange, bool messageOut);
    bool validateRegion_barCode(Mat &region);

};

#endif //OPENNI_GRABBER_DETECTOR_H
