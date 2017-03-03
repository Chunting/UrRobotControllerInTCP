//
// Created by 潘绪洋 on 17-3-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "DriverStatusListener.h"


DriverStatusListener::DriverStatusListener(int argc, char** argv)
        : QtRosNode(argc, argv, "DriverStatusListener"){
}


DriverStatusListener::~DriverStatusListener(){
    COBOT_LOG.info() << "DriverStatusListener released";
}


void DriverStatusListener::ros_comms_init(){
    ros::NodeHandle n;
    robot_status_subscriber = n.subscribe("/driver_status", 20, &DriverStatusListener::robotStatusUpdateCallback, this);
}

void DriverStatusListener::robotStatusUpdateCallback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void DriverStatusListener::run(){
    ros::spin();
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
