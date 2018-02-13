//
// Created by vimjian on 17-4-28.
//

#ifndef LOGISTICS_IMAGECLOUDCONVERTER_H
#define LOGISTICS_IMAGECLOUDCONVERTER_H

#include <vector>
#include "opencv2/opencv.hpp"
#include <Eigen/Geometry>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>

#define K2_CALIB_COLOR         "calib_color.yaml"
#define K2_CALIB_IR            "calib_ir.yaml"
#define K2_CALIB_POSE          "calib_pose.yaml"
#define K2_CALIB_DEPTH         "calib_depth.yaml"

#define K2_CALIB_CAMERA_MATRIX "cameraMatrix"
#define K2_CALIB_DISTORTION    "distortionCoefficients"
#define K2_CALIB_ROTATION      "rotation"
#define K2_CALIB_PROJECTION    "projection"
#define K2_CALIB_TRANSLATION   "translation"
#define K2_CALIB_ESSENTIAL     "essential"
#define K2_CALIB_FUNDAMENTAL   "fundamental"
#define K2_CALIB_DEPTH_SHIFT   "depthShift"

using namespace cv;
using namespace std;

enum ImageMode
{
    HD=1,
    QHD=2
};

class ImageCloudConverter
{
private :
    double fx, fy, cx, cy;
    double depthShift;

    cv::Mat cameraMatrixColor, cameraMatrixDepth,cameraMatrixRegistered,cameraMatrix_qhd;
    cv::Mat distortionColor, distortionDepth;
    cv::Mat rotation, translation;
    Eigen::Matrix4d proj;
    cv::Size sizeRegistered;
    
    cv::Mat lookupX, lookupY;
    float zNear, zFar;

    string path_yaml;

    Mat color,color_qhd;
    Mat color_rect,color_qhd_rect;
    Mat depth,depth_qhd;

    Mat mapX_colorHD,mapY_colorHD;
    Mat mapX_colorQHD,mapY_colorQHD;
    Mat mapX_depth,mapY_depth;
    int mode;

//    pcl::PointCloud<pcl::PointXYZ> cloud;

public:

    ImageCloudConverter();

    void registerDepth( cv::Mat &registeredDepth,cv::Mat &registeredColor);

    void init();

    void ConvertPixel_colorToDepth(const Point2i &colorPixel, Point2i &depthPixel);

    void ConvertPixel_depthToColor(const Point2i &depthPixel, Point2i &colorPixel);

    void ConvertPoint_cloudToColor(const Point3f &cloudPoint, Point2i &colorPixel);

    void ConvertPixel_colorToCloud(const Point2i &colorPixel, Point3f &cloudPoint);

    void ConvertPixel_depthToCloud(const Point2i &depthPixel, Point3f &cloudPoint);

    void ConvertPoint_cloudToDepth(const Point3f &cloudPoint, Point2i &depthPixel);

    void setYamlPath(string path);

    void setInputImages(Mat& rawColor, Mat &rawDepth);
    void setMode(int mode);

private:
    bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;

    bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;

    bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const;

    void remapDepth(const cv::Mat &depth, cv::Mat &scaled) const;

    void projectDepth(const cv::Mat &scaled, cv::Mat &registeredDepth,cv::Mat &registeredColor) const;

    uint16_t interpolate(const cv::Mat &in, const float &x, const float &y) const;

    void createLookup();

    void rectifyColor();

    void initRectify();
};


#endif //LOGISTICS_IMAGECLOUDCONVERTER_H
