//
// Created by 杨帆 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "mainwindow.h"
#include <qapplication.h>

#ifndef QWT_NO_OPENGL
#if QT_VERSION >= 0x040600 && QT_VERSION < 0x050000
#define USE_OPENGL 1
#endif
#endif

#if USE_OPENGL
#include <qgl.h>
#endif
//TODO:LPF滞后，P值提不上去，影响力控性能。考虑使用相位补偿器或卡尔曼滤波器。
//TODO:机器人末端碰撞刚性物体会反弹的问题，考虑使用斜率限制器来解决。
//TODO:考虑将PD控制器改为PID控制器，在I作用下加入随时间衰减的环节。
//TODO:优化界面的修改。
int main(int argc, char** argv) {

    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::FileFinder::addSearchPath("UI");
#if USE_OPENGL
    // on my box QPaintEngine::OpenGL2 has serious problems, f.e:
    // the lines of a simple drawRect are wrong.

    QGL::setPreferredPaintEngine( QPaintEngine::OpenGL );
#endif

    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();
    MainWindow mainWindow;
    mainWindow.resize( 1200, 600 );
    mainWindow.show();
    return a.exec();
}