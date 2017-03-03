//
// Created by 潘绪洋 on 17-1-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QApplication>
#include <cobotsys.h>
#include <cobotsys_file_finder.h>
#include <QtCore/QCommandLineParser>
#include "BinpickingView.h"
#include <Ur3DriverStatusReporter.h>
#include <easy_gui.h>

namespace linux_only {
void remove_shm_obj(){
    cobotsys::easy_gui_reset();
    system("rm -f /dev/shm/*");
}
}

int main(int argc, char** argv){
    linux_only::remove_shm_obj();
    cobotsys::FileFinder::loadDataPaths();
    QApplication a(argc, argv);
    QApplication::setOrganizationName("COBOT");
    QApplication::setApplicationName("binpicking2");

    Ur3DriverStatusReporter reporter(argc, argv, "binpicking2_driver_status_listener");

    QCommandLineParser parser;
    QCommandLineOption debug_model(QStringList() << "d" << "debug", "Show Debug Buttons");

    parser.addOption(debug_model);
    parser.process(a);

    BinpickingView binpickingView;

    reporter.bingGuiApp(a);
    binpickingView.cheat_SetDriverStatusReporter(&reporter);

    if (parser.isSet(debug_model))
        binpickingView.showDebugUi();

    binpickingView.show();

    return a.exec();
}