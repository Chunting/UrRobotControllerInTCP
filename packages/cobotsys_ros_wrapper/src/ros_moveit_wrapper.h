//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_ROS_MOVEIT_WRAPPER_H
#define PROJECT_ROS_MOVEIT_WRAPPER_H

#include <memory>
#include <QString>

class ros_moveit_wrapper {

public:
    ros_moveit_wrapper();
    ~ros_moveit_wrapper();


    bool loadRobotModel(const std::string& robot_description);

protected:
    class MoveItWrapperImpl;
    std::shared_ptr<MoveItWrapperImpl> m_impl;
};


#endif //PROJECT_ROS_MOVEIT_WRAPPER_H
