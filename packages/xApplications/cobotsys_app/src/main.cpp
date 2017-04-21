//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QApplication>
#include "main_window_widget.h"

int main(int argc, char** argv) {
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();

    MainWindow w;
    w.show();
    return a.exec();
}