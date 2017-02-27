//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_compute_node_server.h>
#include <cobotsys_compute_node.h>
#include <cobotsys_background_client.h>
#include <cobotsys_background_server.h>
#include <cobotsys_background_process_client.h>
#include <cobotsys_background_process_server.h>


int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::BackgroundProcessServer app;
    cobotsys::BackgroundProcessClient client;

    if (app.getMaster().lanuchMaster()) {


    } else {

        client.setScriptConfigFile("/home/itolfo/svn/vision/trunk/data/binpicking_action_config.xml");

        client.getSlave().setNodeName("DriverMonitor");
        client.getSlave().connectMaster();
    }

    return a.exec();
}