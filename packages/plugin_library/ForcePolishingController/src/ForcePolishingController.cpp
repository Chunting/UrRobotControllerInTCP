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
    cartesian_path_generator();///< Generate cartesian path of EE
    defineInterimPath();///< insert interim path
    return false;
}

void ForcePolishingController::pause() {

}

void ForcePolishingController::stop() {

}

void ForcePolishingController::cartesian_path_generator() {
	COBOT_LOG.info()<<"The version of CPolishingTask is %s.",m_polishingTask.VERSION_STR.data();
	//Get T_CPi
	std::vector<std::vector<KDL::Frame> >cart_path_cp=m_polishingTask.getCartesianTrajectory();

	//This information is stored in ptd files, CPolishingTask should parse this information and provide it.
	//We simplified T_AB equals to E currently, assume that the Original point of robot is equal to world origin. It can expend if necessary.
	//Input: Output: cartesian_path_
	//T_BC=(T_AB^-1)*T_AE*(T_CP^-1);
	KDL::Frame Frame_AB=m_polishingTask.getRobotFrame();
	for (int i = 0; i < cart_path_cp.size(); ++i) {
		std::vector<KDL::Frame> cart_segment_cp=cart_path_cp[i];
		std::vector<KDL::Frame> cart_segment_bc;
		//Get T_AE. Currently we simplified contact point only have a frame. It will expend to multiple points if necessary.
		KDL::Frame cart_frame_ae=getPolisherFrame(i);
		for (int j = 0; j < cart_segment_cp.size(); ++j) {
			cart_segment_bc.push_back(Frame_AB.Inverse()*cart_frame_ae*cart_segment_cp[j].Inverse());
		}
		cartesian_path_.push_back(cart_segment_bc);
	}
}
KDL::Frame ForcePolishingController::getPolisherFrame(int id) {
	KDL::Frame cart_frame_ae = m_polishingTask.getPolisherFrames().at(0);
	//TODO将旋转角度暂时设置为0.
	std::vector<double> angle_data;
	cart_frame_ae.M = cart_frame_ae.M * KDL::Rotation::RotY(angle_data.at(0));
	return cart_frame_ae;
}
void ForcePolishingController::defineInterimPath() {
	interim_path_.clear();
	long nsize = cartesian_path_.size();
    unsigned long last;
	for (int i = 0; i < nsize; ++i) {
		std::vector<KDL::Frame> frames;
        std::vector<KDL::Frame> intPath;
		last = cartesian_path_[i].size() - 1;
		if (checkPathContinuity(i)) {
            intPath.push_back(cartesian_path_[i][last]);
			COBOT_LOG.notice()<<"path continue" << i;
		} else {
			COBOT_LOG.notice()<<"insert interim path" << i;
            intPath=planInterimPath(i);
		}
		interim_path_.push_back(intPath);
	}
}
bool ForcePolishingController::checkPathContinuity(int path_index) {
	long nsize = cartesian_path_.size();
	long last = cartesian_path_[path_index].size() - 1;

	KDL::Frame last_frame = cartesian_path_[path_index][last], next_frame;
	if (path_index == nsize - 1)
		next_frame = cartesian_path_[0][0];
	else
		next_frame = cartesian_path_[path_index + 1][0];
	if (KDL::Equal(last_frame, next_frame, 2e-3))//2mm
	{
		if (path_index == nsize - 1)
			next_frame = cartesian_path_[0][0];
		else
			next_frame = cartesian_path_[path_index + 1][0];
		return true;
	}
	return false;
}
std::vector<KDL::Frame> ForcePolishingController::planInterimPath(int path_index) {
	long nsize = cartesian_path_.size();
	long last = cartesian_path_[path_index].size() - 1;

	KDL::Frame last_frame = cartesian_path_[path_index][last], next_frame;
	if (path_index == nsize - 1)
		next_frame = cartesian_path_[0][0];
	else
		next_frame = cartesian_path_[path_index + 1][0];

	last_frame.p.data[2] += 0.06;
	next_frame.p.data[2] += 0.06;
	std::vector<KDL::Frame> frames;
	frames.push_back(last_frame);
	frames.push_back(next_frame);
	return frames;
}