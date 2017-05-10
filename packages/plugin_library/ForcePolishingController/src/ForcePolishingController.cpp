//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <QtCore/qjsonarray.h>
#include <extra2.h>
#include "ForcePolishingController.h"
#include <cobotsys_file_finder.h>
#include "frames_io.hpp"
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
	QFile file(m_model_path.c_str());
	file.open(QIODevice::ReadOnly);
	file.seek(0);
	QDataStream in(&file);    // read the data serialized from the file
	//in.setVersion(1);
	in.setByteOrder(QDataStream::LittleEndian);
	in.setFloatingPointPrecision(QDataStream::SinglePrecision);
	quint32 ntriangles;
	quint16 control_bytes;
	file.seek(80);
	in >> ntriangles;
	int triang_count = 0;
	int triang_count50=50;
	double LIMIT_OUT = 2000;
	while (triang_count < ntriangles) {
		float nx, ny, nz, x1, x2, x3, y1, y2, y3, z1, z2, z3;
		file.seek(84+triang_count50+0+0);
		in >> nx;
		file.seek(84+triang_count50+0+4);
		in >> ny;
		file.seek(84+triang_count50+0+8);
		in >> nz;
		file.seek(84+triang_count50+12+0);
		in >> x1;
		file.seek(84+triang_count50+12+4);
		in >> y1;
		file.seek(84+triang_count50+12+8);
		in >> z1;
		file.seek(84+triang_count50+24+0);
		in >> x2;
		file.seek(84+triang_count50+24+4);
		in >> y2;
		file.seek(84+triang_count50+24+8);
		in >> z2;
		file.seek(84+triang_count50+36+0);
		in >> x3;
		file.seek(84+triang_count50+36+4);
		in >> y3;
		file.seek(84+triang_count50+36+8);
		in >> z3;
		file.seek(84+triang_count*50+48);
		in >> control_bytes;
		if (in.status() != QDataStream::Ok ){
			qDebug()<< (int) in.status();
		}
//qDebug() << triangle_i;
		bool nook = false;
		nook = nook || ((fabs((double)x1)) > LIMIT_OUT);
		nook = nook || ((fabs((double)x2)) > LIMIT_OUT);
		nook = nook || ((fabs((double)x3)) > LIMIT_OUT);
		nook = nook || ((fabs((double)y1)) > LIMIT_OUT);
		nook = nook || ((fabs((double)y2)) > LIMIT_OUT);
		nook = nook || ((fabs((double)y3)) > LIMIT_OUT);
		nook = nook || ((fabs((double)z1)) > LIMIT_OUT);
		nook = nook || ((fabs((double)z2)) > LIMIT_OUT);
		nook = nook || ((fabs((double)z3)) > LIMIT_OUT);
		if (control_bytes == 0 && !nook){
//DO TREATMENT
		}
	}

	m_ptd_path=FileFinder::find("CONFIG/ForcePolishing/PolishingTask_test7.ptd");//

	return true;
}

bool ForcePolishingController::start() {
    return false;
}

void ForcePolishingController::pause() {

}

void ForcePolishingController::stop() {

}
