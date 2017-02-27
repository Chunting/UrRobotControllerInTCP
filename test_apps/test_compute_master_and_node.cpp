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



int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);


    cobotsys::BackgroundServer master;
    cobotsys::BackgroundClient slave;

    if (master.lanuchMaster()){

    } else {
        slave.setNodeName("Driver");
        slave.connectMaster();
    }

    return a.exec();
}