//
// Created by zxj on 17-4-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <chrono>
#include <thread>
#include "BinpickingDetectorBox.h"
#include "cobotsys_file_finder.h"

BinpickingDetectorBox::BinpickingDetectorBox() {

}

BinpickingDetectorBox::~BinpickingDetectorBox() {

}


//读取配置文件，设置系统参数
bool BinpickingDetectorBox::setup(const QString &configFilePath) {

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

//    std::string camera_file = "/home/zxj/cobotsys_new/cobotsys/data/Calibration/camera_intrinsic2.yaml";
//    std::string color_matrix_path = "/home/zxj/cobotsys_new/cobotsys/data/Calibration/003926162447/calib_color.yaml";
//    std::string depth_matrix_path = "/home/zxj/cobotsys_new/cobotsys/data/Calibration/003926162447/calib_ir.yaml";
//    std::string hand2eye_file = "/home/zxj/cobotsys_new/cobotsys/data/Calibration/hand2eye_result.xml";

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
        float min_distance_Z = fs["MinDistanceZ"];//工件到相机最小距离   单位：米
        float max_distance_Z = fs["MaxDistanceZ"];//工件到相机最远距离
        float sampling_distance = fs["SamplingDistance"];//点云采样距离
        float plane_ditance = fs["PlaneDitance"];//平面拟合采样距离

        planeSegment = VisionApi::PlaneSegment(min_distance_Z, max_distance_Z, sampling_distance, plane_ditance,
                                               camera_matrix_color, camera_matrix_depth);
        return true;
    } else {
        COBOT_LOG.error() << "can not open system config file! ";
        return false;
    }

}

bool BinpickingDetectorBox::processVisionImage(const std::vector<VisionInputImage> &images) {

    // input check,need at least color and depth image
    if (images.empty()) {
        COBOT_LOG.error() << "processVisionImage: no image input!";
        return false;
    }
    cv::Mat color, depth, color_180, depth_180;
    /*for (int i = 0; i < images.size(); i++) {
        switch (images[i].type) {
            case ImageType::Color:
                color = images[i].image.clone();
                break;
            case ImageType::Depth:
                depth = images[i].image.clone();
                break;
            default:
                break;
        }
    }*/

    color_180 = images[4].image.clone();
    depth_180 = images[1].image.clone();
    cv::flip(color_180,color,1);
    cv::flip(depth_180,depth,1);

    if (color.empty() || depth.empty()) {
        COBOT_LOG.error() << "None color image or depth image!";
        return false;
    }
    COBOT_LOG.debug() << "Image update!";
    depth.convertTo(depth, CV_16UC1);
    planeSegment.processing(color, depth);//检测

    COBOT_LOG.debug() << "Image process finished!";
    debugMat("Color", color);
    debugMat("Depth", 64 * depth);

//    for (int i = 200; i < 230; i++) {
//        for (int j = 200; j < 230; j++) {
//            float depthValue = depth.at<ushort>(i, j);
//            std::cout << setw(5) << depthValue;
//        }
//        std::cout << std::endl;
//    }

//    cv::waitKey(1);

    if (planeSegment.position_pose_vec.empty()) {
        COBOT_LOG.warning() << "None object detected!";
        return false;
    }
    return true;
}

//将计算结果由相机坐标系转到机器人基座坐标系
bool BinpickingDetectorBox::getPickObjects(std::vector<BinObjGrabPose> &result) const {

    result.clear();
    if (planeSegment.position_pose_vec.empty()) {
        COBOT_LOG.warning() << "None object detected!";
        return false;
    }
    for (int i = 0; i < planeSegment.position_pose_vec.size(); i++) {
        BinObjGrabPose binObjGrabPose;
        Eigen::Vector3d coord_in_cam, coord_in_base, normal_in_cam, normal_in_base;
        coord_in_cam[0] = planeSegment.position_pose_vec[i].position.x;
        coord_in_cam[1] = planeSegment.position_pose_vec[i].position.y;
        coord_in_cam[2] = planeSegment.position_pose_vec[i].position.z;

        normal_in_cam = planeSegment.position_pose_vec[i].pose;
        coord_in_base = calbrat.transformPtFromCam2Base(coord_in_cam);
        calbrat.getSuckDirection(normal_in_cam, normal_in_base);

        binObjGrabPose.position.x = coord_in_base[0];
        binObjGrabPose.position.y = coord_in_base[1];
        binObjGrabPose.position.z = coord_in_base[2];
//
//        binObjGrabPose.rotation[0] = normal_in_base[0];//TODO  四元素转RPY
//        binObjGrabPose.rotation[1] = normal_in_base[1];
//        binObjGrabPose.rotation[2] = normal_in_base[2];

//        binObjGrabPose.position.x = 0.47565;
//        binObjGrabPose.position.y = -0.91442;
//        binObjGrabPose.position.z = 0.60564;

        binObjGrabPose.rotation[0] = -CV_PI;
        binObjGrabPose.rotation[1] = 0;
        binObjGrabPose.rotation[2] = -CV_PI / 2;

        binObjGrabPose.target_info = "box";
        result.push_back(binObjGrabPose);

//#ifdef DEBUG
        COBOT_LOG.debug() << "相机坐标系坐标："
                          << std::setw(15) << coord_in_cam[0]
                          << std::setw(15) << coord_in_cam[1]
                          << std::setw(15) << coord_in_cam[2];

        COBOT_LOG.debug() << "基座坐标系坐标："
                          << std::setw(15) << coord_in_base[0]
                          << std::setw(15) << coord_in_base[1]
                          << std::setw(15) << coord_in_base[2];


        Eigen::Vector3d vz(0, 0, 1);//TODO 法向量角度判断
        Eigen::Vector3d pose;
        pose[0] = normal_in_base[0];
        pose[1] = normal_in_base[1];
        pose[2] = normal_in_base[2];

        double Ndegree = acos(vz.dot(pose) / pose.norm()) * 180 / CV_PI;
        std::cout << "------>法向量和z轴的夹角:" << Ndegree << std::endl;
//#endif
    }
//    result.clear();
    return true;
}
