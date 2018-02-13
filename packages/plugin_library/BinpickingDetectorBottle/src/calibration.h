//
// Created by zxj on 16-12-23.
// implement and clear up by zsh
//

#ifndef LIBCOBOTSYS_CALIBRATION_H
#define LIBCOBOTSYS_CALIBRATION_H

#include <cxcore.h>
#include <Eigen/Dense>
#include "pinhole_camera_model.h"

namespace cobotsys {

class calibration {
    //构造与析构函数
public:
    calibration();
    ~calibration();

    //接口函数
public:
    Eigen::Vector3d upProjectPt(double x, double y, double d)  ;//将图像点反投影为相机坐标系中的点。
    Eigen::Vector3d unProjectPt(const cv::Point2d &ptImg, double d);//将图像点反投影为相机坐标系中的点。
    cv::Point2d projectPt(const Eigen::Vector3d &ptCam);//将相机坐标系中的点投影为图像点。
    Eigen::Vector3d transformPtFromCam2Base(const Eigen::Vector3d & ptCam) const ;//将相机坐标系中的点变换至基座坐标系。
    Eigen::Vector3d transformPtFromImg2Base(const cv::Point2d &ptImg, double d);//将图像坐标转化为基座坐标系内的点。
    Eigen::Vector3d transformPtFromBase2Cam(const Eigen::Vector3d & ptBase);//将基座坐标系中的点变换至相机坐标系。
    cv::Point2d transformPtFromBase2Img(const Eigen::Vector3d &ptBase);//将基座坐标系内的点变换至相机坐标系。

    bool loadCameraModel(const std::string &camera_coeff_yaml_file);
    bool loadHandEyeData(const std::string &hand2eye_cal_fileName);
    bool calObjPlaneNormalOffset();
    bool getBestSuckPosandDirection(Eigen::Quaterniond &q_rot_obj2cam,
                                    Eigen::Vector3d &t_obj2cam,
                                    Eigen::Vector3d &out_suckOri,
                                    Eigen::Vector3d &out_suckPosition)  ;

    Eigen::Matrix4d combineRotT(Eigen::Quaternion<double> q_rot, Eigen::Vector3d t) const ;

    bool getSuckDirection(const Eigen::Vector3d& in_planeNorm,Eigen::Vector3d& out_planeNorm_in_base) ;

    //内部函数
private:
    bool getBestSuckPosDirection(Eigen::Quaterniond &q_rot_obj2cam,
                                 Eigen::Vector3d &t_obj2cam,
                                 Eigen::Matrix<double, 4, 6> &plane_normal_obj,
                                 std::vector<double> &ObjPlaneCenteroffset,
                                 Eigen::Quaterniond &q_rot_cam2base,
                                 Eigen::Vector3d &t_cam2base,
                                 Eigen::Vector3d &suckOri,
                                 Eigen::Vector3d &suckPosition)  ;

private:
    //手眼标定数据
    Eigen::Quaterniond q_rot_cam2base_;
    Eigen::Vector3d t_cam2base_;
    Eigen::Matrix<double, 4, 6> plane_normal_obj_;
    std::vector<double> plane_center_offset_obj_;

    //相机标定数据
    cv_ext::PinholeCameraModel cameraModel_;
    std::string cameraModelFile_;
    int imageWidth_;
    int imageHeight_;
    double imageScaleFactor_;
};
}

#endif //LIBCOBOTSYS_CALIBRATION_H
