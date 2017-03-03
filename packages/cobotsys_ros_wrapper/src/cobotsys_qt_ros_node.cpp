//
// Created by 潘绪洋 on 17-3-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_qt_ros_node.h"
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <cobotsys_logger.h>


namespace cobotsys {


QtRosNode::QtRosNode(int argc, char** argv, const std::string& name) :
        init_argc(argc),
        init_argv(argv),
        node_name(name){}

QtRosNode::~QtRosNode(){
    shutdown();
    COBOT_LOG.info() << "QtRosNode Released";
}

/**
 * This is called by the qt application to stop the ros node before the
 * qt app closes.
 */
void QtRosNode::shutdown(){
    if (ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QtRosNode::on_init(){
    ros::init(init_argc, init_argv, node_name);
    if (!ros::master::check()) {
        COBOT_LOG.warning() << "No Master";
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros_comms_init();
    start();
    return true;
}

bool QtRosNode::on_init(const std::string& master_url, const std::string& host_url){
    std::map<std::string, std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings, node_name);
    if (!ros::master::check()) {
        return false;
    }
    ros::start(); // our node handles go out of scope, so we want to control shutdown explicitly.
    ros_comms_init();
    start();
    return true;
}

void QtRosNode::bindApp(QCoreApplication& app){
    COBOT_LOG.notice() << "bindApp: QCoreApplication";
    app.connect(&app, &QCoreApplication::aboutToQuit, this, &QtRosNode::shutdown);
}

void QtRosNode::bingGuiApp(QApplication& app){
    COBOT_LOG.notice() << "bindApp: QApplication";
    app.connect(&app, &QApplication::lastWindowClosed, this, &QtRosNode::shutdown);
}

const std::string& QtRosNode::nodeName() const{
    return node_name;
}


//
}