//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtGui>
#include <QApplication>
#include <DriverStatusListener.h>
#include <QtWidgets/QWidget>

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char** argv){

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);

    DriverStatusListener listener(argc, argv);

    if (listener.on_init()) {
        app.connect(&app, &QCoreApplication::aboutToQuit, &listener, &DriverStatusListener::shutdown);

        QWidget w;
        w.show();

        return app.exec();
    }

    return 1;
}