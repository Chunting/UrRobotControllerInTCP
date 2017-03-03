//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "Ur3DriverStatusReporter.h"

Ur3DriverStatusReporter::Ur3DriverStatusReporter(int argc, char** argv, const char* name)
        : QtRosNode(argc, argv, name){
}

Ur3DriverStatusReporter::~Ur3DriverStatusReporter(){
}

void Ur3DriverStatusReporter::ros_comms_init(){
    ros::NodeHandle n;
    robot_status_subscriber = n.subscribe("/driver_status", 20, &Ur3DriverStatusReporter::robotControlStatusCallback,
                                          this);
}

void Ur3DriverStatusReporter::robotControlStatusCallback(const std_msgs::String::ConstPtr& msg){
    COBOT_LOG.info() << "Status Updated: " << msg->data;
    Q_EMIT robotControlStatusUpdated(QString::fromLocal8Bit(msg->data.c_str()));
}

void Ur3DriverStatusReporter::run(){
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
