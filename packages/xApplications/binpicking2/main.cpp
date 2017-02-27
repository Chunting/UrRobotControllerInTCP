//
// Created by 潘绪洋 on 17-1-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QApplication>
#include <cobotsys.h>
#include <cobotsys_file_finder.h>
#include "BinpickingView.h"

namespace linux_only {
void remove_shm_obj(){
    system("rm -f /dev/shm/*");
}
}

int main(int argc, char **argv){
    linux_only::remove_shm_obj();
    cobotsys::FileFinder::loadDataPaths();
    QApplication a(argc, argv);
    QApplication::setOrganizationName("COBOT");
    QApplication::setApplicationName("binpicking2");

    BinpickingView bpv;
    bpv.show();

    return a.exec();
}