//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
//#include <QtCore/QJsonObject>
//#include <QtCore/qjsonarray.h>
#include <extra2.h>
#include "ForcePolishingController.h"
#include <cobotsys_file_finder.h>

using namespace cobotsys;
using namespace std;
using namespace KDL;
ForcePolishingController::ForcePolishingController(){

}
ForcePolishingController::~ForcePolishingController(){

}

bool ForcePolishingController::setup(const QString& configFilePath) {
    //TODO 暂时使用默认路径，导入ptd文件路径和stl文件路径。
	m_model_path=FileFinder::find("CONFIG/ForcePolishing/phone_shell_bin.stl");
	m_polishingTask.parseSTL(m_model_path);
	m_ptd_path=FileFinder::find("CONFIG/ForcePolishing/PolishingTask_test7.ptd");//
	m_polishingTask.parsePTD(m_ptd_path);
	return true;
}

bool ForcePolishingController::start() {
    return false;
}

void ForcePolishingController::pause() {

}

void ForcePolishingController::stop() {

}
