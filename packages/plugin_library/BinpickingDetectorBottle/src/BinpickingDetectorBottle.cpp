//
// Created by zxj on 17-4-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <chrono>
#include <thread>
#include "BinpickingDetectorBottle.h"
#include "cobotsys_file_finder.h"

BinpickingDetectorBottle::BinpickingDetectorBottle() {

}

BinpickingDetectorBottle::~BinpickingDetectorBottle() {

}


//读取配置文件，设置系统参数
bool BinpickingDetectorBottle::setup(const QString &configFilePath) {

    auto config = configFilePath.toStdString();
    auto configFilePath_ = FileFinder::find(config);

    cv::Mat camera_matrix_color = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat camera_matrix_depth = cv::Mat::zeros(3, 3, CV_64F);

    cobotsys::FileFinder::loadDataPaths();
    cobotsys::FileFinder::addSearchPath("Calibration");
    cobotsys::FileFinder::addSearchPath("/../Calibration");
    cobotsys::FileFinder::addSearchPath("Calibration/003992562447");
    cobotsys::FileFinder::dumpAvailablePath();
    std::string camera_file = cobotsys::FileFinder::find("camera_intrinsic.yaml");
    std::string color_matrix_path = cobotsys::FileFinder::find("calib_color.yaml");
    std::string depth_matrix_path = cobotsys::FileFinder::find("calib_ir.yaml");
    std::string hand2eye_file = cobotsys::FileFinder::find("hand2eye_result.xml");

    if (!calbrat.loadHandEyeData(hand2eye_file)) {
        COBOT_LOG.error() << "Failed to load hand2eye params";
        return false;
    }

    if (!calbrat.loadCameraModel(camera_file)) {
        COBOT_LOG.error() << "Failed to load camera intrinsic";
        return false;
    }

    cv::FileStorage fs_color(color_matrix_path, cv::FileStorage::READ);//彩色相机内参
    if (fs_color.isOpened()) {
        fs_color["cameraMatrix"] >> camera_matrix_color;
    } else {
        COBOT_LOG.error() << "can not open color calibration file! ";
        return false;
    }

    cv::FileStorage fs_depth(depth_matrix_path, cv::FileStorage::READ);//深度相机内参
    if (fs_depth.isOpened()) {
        fs_depth["cameraMatrix"] >> camera_matrix_depth;
    } else {
        COBOT_LOG.error() << "can not open depth calibration file! ";
        return false;
    }

    cv::FileStorage fs(configFilePath_, cv::FileStorage::READ);
    if (fs.isOpened()) {
        return true;
    } else {
        COBOT_LOG.error() << "can not open system config file! ";
        return false;
    }

}

bool BinpickingDetectorBottle::processVisionImage(const std::vector<VisionInputImage> &images) {

    // input check,need at least color and depth image
    if (images.empty()) {
        COBOT_LOG.error() << "processVisionImage: no image input!";
        return false;
    }
    cv::Mat color, depth, color_180, depth_180;

    color_180 = images[0].image.clone();
    depth = images[1].image.clone();
    cv::flip(color_180,color,1);
//    cv::flip(depth_180,depth,1);

    if (color.empty() || depth.empty()) {
        COBOT_LOG.error() << "None color image or depth image!";
        return false;
    }
    COBOT_LOG.debug() << "Image update!";
    //depth.convertTo(depth, CV_16UC1);

    detector.SetColorImage(color);
    detector.FindGraspRegion_combine_label_barCode(color);

    COBOT_LOG.debug() << "Image process finished!";
    //debugMat("Color", color);
    //debugMat("Depth", 64 * depth);
    //cv::waitKey(1);

    if (detector.candidatePoints.empty()) {
        COBOT_LOG.warning() << "None object detected!";
        return false;
    }

    positions.clear();
    for (int i = 0; i < detector.candidatePoints.size(); i++) {
        Eigen::Vector3d t_camera, t_robot;
        t_camera[0] = detector.candidatePoints[i].x;
        t_camera[1] = detector.candidatePoints[i].y;
        t_camera[2] = 0.92;
        Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
        Eigen::Quaterniond q_rot_obj2cam(m);
        Eigen::Vector3d out_suckOri;
        Eigen::Vector3d t_obj2cam = calbrat.upProjectPt(t_camera[0]+100, t_camera[1]+100, t_camera[2]);
        calbrat.getBestSuckPosandDirection(q_rot_obj2cam, t_obj2cam, out_suckOri, t_robot);
        positions.push_back(t_robot);
    }
    return true;
}

//将计算结果由相机坐标系转到机器人基座坐标系
bool BinpickingDetectorBottle::getPickObjects(std::vector<BinObjGrabPose> &result) const {

    result.clear();
    if (positions.empty()) {
        COBOT_LOG.warning() << "None object detected!";
        return false;
    }

    for (int i = 0; i < positions.size(); i++) {
        BinObjGrabPose binObjGrabPose;
        binObjGrabPose.position.x = positions[i][0];
        binObjGrabPose.position.y = positions[i][1];
        binObjGrabPose.position.z = positions[i][2];

        binObjGrabPose.rotation[0] = -CV_PI;
        binObjGrabPose.rotation[1] = 0;
        binObjGrabPose.rotation[2] = -CV_PI / 2;

        binObjGrabPose.target_info = "bottle";
        result.push_back(binObjGrabPose);

        COBOT_LOG.debug() << "基座坐标系坐标："
                          << std::setw(15) << positions[i][0]
                          << std::setw(15) << positions[i][1]
                          << std::setw(15) << positions[i][2];
    }
   // result.clear();
    return true;
}
