//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_compute_master.h>
#include <cobotsys_compute_node.h>



int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);


    cobotsys::ComputeMaster master;
    cobotsys::ComputeNode node;

    if (master.lanuchMaster()){

    } else {
        node.connectMaster();
    }

    return a.exec();
}