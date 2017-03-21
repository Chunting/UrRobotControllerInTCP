//
// Created by 潘绪洋 on 17-3-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <ros_moveit_wrapper.h>
#include <ros/spinner.h>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros_moveit_wrapper rosMoveitWrapper;
    rosMoveitWrapper.loadRobotModel("robot_description");

    ros::shutdown();
    return 0;
}