//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <QtCore/QJsonObject>
#include <QtCore/qjsonarray.h>
#include <extra2.h>
#include "KinematicSolver.h"
#include <cobotsys_file_finder.h>
using namespace cobotsys;
KinematicSolver::KinematicSolver(){

}

KinematicSolver::~KinematicSolver(){

}

bool KinematicSolver::solve(const vector<double>& cur, const vector<double>& target, vector<double>& result) {
	return true;
}
bool KinematicSolver::setup(const QString& configFilePath) {
	//auto a = FileFinder::find(configFilePath.toStdString);
    //load model from json file.
    //TODO 后期需要json文件的有效性验证功能。
	COBOT_LOG.info() << "configFilePath" <<configFilePath.toStdString();
    QJsonObject json;
    if (loadJson(json, configFilePath)) {	
        foreach (const QJsonValue & value, json["chain"].toArray()) {
            QJsonObject segmentObj = value.toObject();
            std::string segmentName=segmentObj["name"].toString().toStdString();

            QJsonArray rpy=segmentObj["frame"].toObject()["rpy"].toArray();
			//COBOT_LOG.info() << "rpy count:" << rpy.count();
			//COBOT_LOG.info() << "rpy.at(0).toDouble():" <<segmentObj["frame"].toObject()["rpy"].toArray().at(0).isDouble();
			//COBOT_LOG.info() << "rpy.at(1).toDouble():" << segmentObj["frame"].toObject()["rpy"].toArray()[0].toDouble();
			//COBOT_LOG.info() << "rpy.at(2).toDouble():" << rpy.at(2).toDouble();
            KDL::Rotation rot=KDL::Rotation::RPY(rpy.at(0).toDouble(), rpy.at(1).toDouble(), rpy.at(2).toDouble());

            QJsonArray xyz=segmentObj["frame"].toObject()["xyz"].toArray();
			
			KDL::Vector vec(xyz.at(0).toDouble(), xyz.at(1).toDouble(), xyz.at(2).toDouble());;

            //inline Frame(const Rotation& R,const Vector& V);
            KDL::Frame frame(rot,vec);

            int axis_xyz=segmentObj["rotation"].toObject()["axis_xyz"].toInt();
            KDL::Joint joint((KDL::Joint::JointType)axis_xyz);
            KDL::Segment seg = KDL::Segment(segmentName,joint,frame);
            m_robot_chain.addSegment(seg);
            JointLimits jointLimits;
            jointLimits.lower=segmentObj["rotation"].toObject()["limit"].toObject()["lower"].toDouble();
            jointLimits.upper=segmentObj["rotation"].toObject()["limit"].toObject()["upper"].toDouble();
            jointLimits.effort=segmentObj["rotation"].toObject()["limit"].toObject()["effort"].toDouble();
            jointLimits.velocity=segmentObj["rotation"].toObject()["limit"].toObject()["velocity"].toDouble();
            m_robot_joint_limits.push_back(jointLimits);
        }
		//从1到6为机器人六个可活动关节,其余limit值无效。
		m_robot_joint_limits.erase(m_robot_joint_limits.begin());
		m_robot_joint_limits.erase(m_robot_joint_limits.end()-1);
        return true;
    }
    return false;
}
