//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ROS_MOVEIT_WRAPPER_H
#define PROJECT_ROS_MOVEIT_WRAPPER_H

#include <memory>
#include <string>
#include <QString>
#include <opencv/cv.h>

class ros_moveit_wrapper {

public:
    ros_moveit_wrapper();
    ~ros_moveit_wrapper();


    bool loadRobotModel(const std::string& robot_description = "robot_description");

    bool getIK(const cv::Point3d& pos, const cv::Vec3d& normal, std::vector<double>& jointValue);

    void forwardKinematics(const std::vector<double>& jointValue);

protected:
    class MoveItWrapperImpl;
    std::shared_ptr<MoveItWrapperImpl> m_impl;
};


#endif //PROJECT_ROS_MOVEIT_WRAPPER_H
