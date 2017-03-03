//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DriverStatusPublisher.h"


DriverStatusPublisher::DriverStatusPublisher(int argc, char** argv)
        : QtRosNode(argc, argv, "DriverStatusPublisher"){
}

DriverStatusPublisher::~DriverStatusPublisher(){
}

void DriverStatusPublisher::ros_comms_init(){
    ros::NodeHandle n;
    robot_status_publisher = n.advertise<std_msgs::String>("driver_status", 5);
}

void DriverStatusPublisher::run(){
    ros::Rate loop_rate(1);
    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "hello world " << std::setw(5) << count << ", " << ros::master::check();
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        robot_status_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
