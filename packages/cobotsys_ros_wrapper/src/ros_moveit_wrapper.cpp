//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include "ros_moveit_wrapper.h"


class ros_moveit_wrapper::MoveItWrapperImpl {
public:
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    bool loadModel(const std::string& robot_description){
        robot_model_loader::RobotModelLoader robot_model_loader(robot_description);
        kinematic_model = robot_model_loader.getModel();
        ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

        kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
        kinematic_state->setToDefaultValues();

        return true;
    }
};

ros_moveit_wrapper::ros_moveit_wrapper()
        : m_impl(new MoveItWrapperImpl){
}

ros_moveit_wrapper::~ros_moveit_wrapper(){
}

bool ros_moveit_wrapper::loadRobotModel(const std::string& robot_description){
    return false;
}
