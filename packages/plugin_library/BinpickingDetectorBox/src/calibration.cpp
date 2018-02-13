//
// Created by zxj on 16-12-23.
// implement and clear up by zsh
//

#include "calibration.h"
#include <cobotsys.h>
#include <iostream>
#include <cobotsys_file_finder.h>

namespace other {
    Eigen::Quaterniond RotationYXZ(double angleX, double angleY, double angleZ) {
        double _[9];
        double sx, cx, sy, cy, sz, cz;
        sx = sin(angleX);
        cx = cos(angleX);
        sy = sin(angleY);
        cy = cos(angleY);
        sz = sin(angleZ);
        cz = cos(angleZ);
        _[0 * 3 + 0] = cy * cz + sx * sy * sz;
        _[0 * 3 + 1] = -cy * sz + sx * sy * cz;
        _[0 * 3 + 2] = cx * sy;
        _[1 * 3 + 0] = cx * sz;
        _[1 * 3 + 1] = cx * cz;
        _[1 * 3 + 2] = -sx;
        _[2 * 3 + 0] = -sy * cz + sx * cy * sz;
        _[2 * 3 + 1] = sy * sz + sx * cy * cz;
        _[2 * 3 + 2] = cx * cy;

        Eigen::Matrix3d matrix3d;
        matrix3d << _[0], _[1], _[2]
                , _[3], _[4], _[5]
                , _[6], _[7], _[8];

        Eigen::Quaterniond q(matrix3d);
        return q;
    }
}

namespace cobotsys {
    calibration::calibration() {
        std::vector<double> offset{0, 0, 0, 0, 0, 0};
        plane_center_offset_obj_ = offset;

        plane_normal_obj_ << 1, -1, 0, 0, 0, 0,
                0, 0, 1, -1, 0, 0,
                0, 0, 0, 0, 1, -1,
                0, 0, 0, 0, 0, 0;
    }

    calibration::~calibration() {

    }
//
//
//接口函数
    cv::Point2d calibration::projectPt(const Eigen::Vector3d &scenePt) {
        double imgPt[2] = {-1, -1};
        double sPt[3] = {scenePt(0), scenePt(1), scenePt(2)};
        cameraModel_.project(sPt, imgPt);
        cv::Point2d v;
        v.x = imgPt[0];
        v.y = imgPt[1];
        return v;
    }

    Eigen::Vector3d calibration::unProjectPt(const cv::Point2d &ptImg, double d) {
        return upProjectPt(ptImg.x, ptImg.y, d);
    }

    Eigen::Vector3d calibration::upProjectPt(double x, double y, double d) {
        double imgPt[2] = {x, y};
        double scenePt[3];
        cameraModel_.unproject(imgPt, d, scenePt);
        Eigen::Vector3d v(scenePt);
        return v;
    }

    Eigen::Vector3d calibration::transformPtFromCam2Base(const Eigen::Vector3d &ptCam) const {

        Eigen::Matrix4d m_proj_cam2base = combineRotT(q_rot_cam2base_, t_cam2base_);
        Eigen::Vector4d p;
        p.block(0, 0, 3, 1) = ptCam;
        p(3) = 1;
        Eigen::Vector4d pInBase = m_proj_cam2base * p;
        return pInBase.block(0, 0, 3, 1);
    }

    Eigen::Vector3d calibration::transformPtFromImg2Base(const cv::Point2d &ptImg, double d) {
        Eigen::Vector3d ptInCam = unProjectPt(ptImg, d);
        return transformPtFromCam2Base(ptInCam);
    }

    Eigen::Vector3d calibration::transformPtFromBase2Cam(const Eigen::Vector3d &ptBase) {
        Eigen::Matrix4d m_proj_cam2base = combineRotT(q_rot_cam2base_, t_cam2base_);
        Eigen::Vector4d p;
        p.block(0, 0, 3, 1) = ptBase;
        p(3) = 1;
        Eigen::Vector4d pInCam = m_proj_cam2base.inverse() * p;
        return pInCam.block(0, 0, 3, 1);
    }

    cv::Point2d calibration::transformPtFromBase2Img(const Eigen::Vector3d &ptBase) {
        Eigen::Vector3d ptInCam = transformPtFromBase2Cam(ptBase);
        return projectPt(ptInCam);
    }

    bool calibration::getBestSuckPosandDirection(Eigen::Quaterniond &q_rot_obj2cam,
                                                 Eigen::Vector3d &t_obj2cam,
                                                 Eigen::Vector3d &out_suckOri,
                                                 Eigen::Vector3d &out_suckPosition) {

        return getBestSuckPosDirection(q_rot_obj2cam,
                                       t_obj2cam,
                                       plane_normal_obj_,
                                       plane_center_offset_obj_,
                                       q_rot_cam2base_,
                                       t_cam2base_,
                                       out_suckOri,
                                       out_suckPosition);
    }
//

//
//内部函数
    bool calibration::getBestSuckPosDirection(Eigen::Quaterniond &q_rot_obj2cam,
                                              Eigen::Vector3d &t_obj2cam,
                                              Eigen::Matrix<double, 4, 6> &plane_normal_obj,
                                              std::vector<double> &ObjPlaneCenteroffset,
                                              Eigen::Quaterniond &q_rot_cam2base,
                                              Eigen::Vector3d &t_cam2base,
                                              Eigen::Vector3d &suckOri,
                                              Eigen::Vector3d &suckPosition) {
        //to get a quaternion to describe the direction of end effector in base frame.
        //to get the quaternion, we need to calculate each normal of the object's planes
        //that can be sucked, then get the one most upward.

        //get the appropriate plane normal in base,  A,B,C,D . four plane can be sucked
        //first get the best plane, then get the center of the plane.
        Eigen::Matrix<double, 4, 6> plane_normal_obj_in_base;
        Eigen::Vector3d basez(0, 0, 1);
        q_rot_obj2cam.normalize();
        Eigen::Matrix4d m_proj_obj2cam = combineRotT(q_rot_obj2cam, t_obj2cam);
        q_rot_cam2base.normalize();
        Eigen::Matrix4d m_proj_cam2base = combineRotT(q_rot_cam2base, t_cam2base);
        Eigen::Matrix4d totalTrans = m_proj_cam2base * m_proj_obj2cam;
        plane_normal_obj_in_base = totalTrans * plane_normal_obj;
        Eigen::Matrix<double, 3, 6> plane_normal = plane_normal_obj_in_base.block(0, 0, 3, 6);
        for (int j = 0; j < plane_normal.cols(); j++) {
            plane_normal.col(j) /= plane_normal.col(j).norm();
        }
        std::vector<double> dotRes;
        for (int i = 0; i < plane_normal.cols(); i++) {
            dotRes.push_back(plane_normal.col(i).dot(basez));
        }

        int maxIdx = std::distance(dotRes.begin(), std::max_element(dotRes.begin(), dotRes.end()));

        suckOri = -plane_normal.col(maxIdx);
        suckOri = suckOri;
        double offset = ObjPlaneCenteroffset[maxIdx];
        suckPosition = suckOri * offset + totalTrans.block(0, 3, 3, 1);

        return true;

    }
//
    bool
    calibration::getSuckDirection(const Eigen::Vector3d &in_planeNorm, Eigen::Vector3d &out_planeNorm_in_base) const {
        Eigen::Vector4d normIncam(0, 0, 0, 0);
        normIncam(0) = -in_planeNorm(0);
        normIncam(1) = -in_planeNorm(1);
        normIncam(2) = -in_planeNorm(2);
        Eigen::Matrix4d m_proj_cam2base = combineRotT(q_rot_cam2base_, t_cam2base_);
        Eigen::Vector4d normInbase = m_proj_cam2base * normIncam;
        out_planeNorm_in_base = normInbase.block(0, 0, 3, 1);
    }


    bool calibration::loadHandEyeData(const std::string &hand2eye_cal_fileName) {//给定手眼标定文件的完整目录

        cv::FileStorage fs(hand2eye_cal_fileName, cv::FileStorage::READ);
        if (fs.isOpened()) {

            q_rot_cam2base_.x() = fs["RotationX"];
            q_rot_cam2base_.y() = fs["RotationY"];
            q_rot_cam2base_.z() = fs["RotationZ"];
            q_rot_cam2base_.w() = fs["RotationW"];

            t_cam2base_.x() = fs["TranslationX"];
            t_cam2base_.y() = fs["TranslationY"];
            t_cam2base_.z() = fs["TranslationZ"];

            return true;
        } else {
            return false;
        }
    }

    bool calibration::calObjPlaneNormalOffset() {

        return true;
    }

//zsh
    bool calibration::loadCameraModel(const std::string &camera_coeff_yaml_file) {
        cv::FileStorage fs(camera_coeff_yaml_file, cv::FileStorage::READ);
        if (fs.isOpened()) {
            cameraModelFile_ = camera_coeff_yaml_file;

            cv::Mat camera_matrix = cv::Mat_<double>(3, 3);
            cv::Mat dist_coeff = cv::Mat_<double>(5, 1);

            fs["cameraMatrix"] >> camera_matrix;
            fs["distortionCoefficients"] >> dist_coeff;
            if (dist_coeff.rows == 1)
                dist_coeff = dist_coeff.t();

            fs["imageWidth"] >> imageWidth_;
            fs["imageHeight"] >> imageHeight_;
            fs["imageScaleFactor"] >> imageScaleFactor_;

//            cout << setw(20) << "CameraMatrix: " << endl << camera_matrix << endl
//                 << setw(20) << "CameraDistCoeff:" << dist_coeff << endl
//                 << setw(20) << "imageWidth:" << imageWidth_ << endl
//                 << setw(20) << "imageHeight:" << imageHeight_ << endl
//                 << setw(20) << "imageScaleFactor:" << imageScaleFactor_ << endl
//                 << endl;

            cameraModel_ = cv_ext::PinholeCameraModel(camera_matrix,
                                                      imageWidth_,
                                                      imageHeight_,
                                                      imageScaleFactor_,
                                                      dist_coeff);
            return true;
        }
        return false;
    }

    Eigen::Matrix4d calibration::combineRotT(Eigen::Quaternion<double> q_rot, Eigen::Vector3d t) const {
        Eigen::Matrix4d m4;
        Eigen::Matrix3d m_rot = q_rot.toRotationMatrix();
        m4.block(0, 0, 3, 3) = m_rot;
        m4.row(3) << 0, 0, 0, 1;
        m4.col(3) << t(0), t(1), t(2), 1;
        return m4;
    }


}///// end namespace cobotsys