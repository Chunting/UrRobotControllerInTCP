//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DriverStatusPublisher.h"
#include <cobotsys_logger.h>

DriverStatusPublisher::DriverStatusPublisher(int argc, char** argv)
        : QtRosNode(argc, argv, "DriverStatusPublisher"){
}

DriverStatusPublisher::~DriverStatusPublisher(){
    COBOT_LOG.info() << "DriverStatusPublisher released";
}

void DriverStatusPublisher::ros_comms_init(){
    ros::NodeHandle n;
    robot_status_publisher = n.advertise<std_msgs::String>("driver_status", 5);
}

void DriverStatusPublisher::run(){
    ros::Rate loop_rate(1);
    int count = 0;

    std::vector<std::string> robot_status_array;

    robot_status_array.push_back("grab_image");
    robot_status_array.push_back("error");

    while (ros::ok()) {
        std_msgs::String msg;

        msg.data = robot_status_array[count % robot_status_array.size()];

        ROS_INFO("%s", msg.data.c_str());

        robot_status_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
