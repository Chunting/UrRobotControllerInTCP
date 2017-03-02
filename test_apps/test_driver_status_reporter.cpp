//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys.h>
#include <QtCore/QCoreApplication>
#include <cobotsys_simple_network_server.h>
#include <cobotsys_simple_network_client.h>
#include <cobotsys_background_json_client.h>
#include <cobotsys_background_json_server.h>
#include <cobotsys_background_process_client.h>
#include <cobotsys_file_finder.h>
#include <thread>


/// 虽然实现了在另一个线程里创建并调用QtCore。但是不推荐。
void runsInOtherThread(int argc, char* argv[]){
    QCoreApplication a(argc, argv);
    cobotsys::init_library(argc, argv);

    auto file = cobotsys::FileFinder::find("binpicking_action_config.xml");
    cobotsys::BackgroundProcessClient client;
    client.setScriptConfigFile(file.c_str());

    client.getSlave().setNodeName("Driver");
    client.getSlave().connectHost();
    a.exec();
}


int main(int argc, char** argv){
    std::thread t(runsInOtherThread, argc, argv);

    char c;
    c = getchar();

    while (c != 'q') {
        c = getchar();
    }
    QCoreApplication::exit(0);
    return 0;
}