//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_compute_master.h>
#include <cobotsys_compute_node.h>
#include <cobotsys_background_slave.h>
#include <cobotsys_background_master.h>



int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);


    cobotsys::BackgroundMaster master;
    cobotsys::BackgroundSlave slave;

    if (master.lanuchMaster()){

    } else {
        slave.setNodeName("Driver");
        slave.connectMaster();
    }

    return a.exec();
}