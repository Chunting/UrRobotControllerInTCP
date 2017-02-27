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
#include <cobotsys_file_finder.h>


int main(int argc, char **argv){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    auto file = cobotsys::FileFinder::find("binpicking_action_config.xml");
    cobotsys::BackgroundProcessClient client;
    client.setScriptConfigFile(file.c_str());

    client.getSlave().setNodeName("Driver");
    client.getSlave().connectMaster();

    return a.exec();
}