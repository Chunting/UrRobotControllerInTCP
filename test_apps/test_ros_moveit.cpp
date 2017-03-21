//
// Created by 潘绪洋 on 17-3-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "ros_moveit_wrapper.h"
#include <ros/spinner.h>
#include <ros/ros.h>

/**
 * 要运行起来这个程序，得首先ur3-moveit相关的驱动都安装上。
 * 然后在命令行里运行
 * roslaunch ur3_moveit_config planning_context.launch load_robot_description:=true
 * 这样就可以有默认的机器人参数了。
 */

int main(int argc, char** argv){
    ros::init(argc, argv, "arm_kinematics");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros_moveit_wrapper rosMoveitWrapper;
    rosMoveitWrapper.loadRobotModel("robot_description");

    ros::shutdown();
    return 0;
}