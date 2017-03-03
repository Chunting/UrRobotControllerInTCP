//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRIVERSTATUSPUBLISHER_H
#define PROJECT_DRIVERSTATUSPUBLISHER_H

#include <cobotsys_qt_ros_node.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

class DriverStatusPublisher : public cobotsys::QtRosNode {
public:
    DriverStatusPublisher(int argc, char** argv);
    virtual ~DriverStatusPublisher();

    void run();
    void ros_comms_init();
private:
    ros::Publisher robot_status_publisher;
};


#endif //PROJECT_DRIVERSTATUSPUBLISHER_H
