//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <cobotsys_message_server.h>
#include <QtCore/QCoreApplication>

int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    cobotsys::distributed_system::MessageServer::lanuchServer();
    return a.exec();
}