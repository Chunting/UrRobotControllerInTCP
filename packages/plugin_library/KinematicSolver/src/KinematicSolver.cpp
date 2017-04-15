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
#include "../orocos_kdl/src/frames_io.hpp"
using namespace cobotsys;
using namespace std;
using namespace KDL;
KinematicSolver::KinematicSolver(){

}

KinematicSolver::~KinematicSolver(){

}
int KinematicSolver::cartToJnt(const std::vector<double>& initialJoint, const std::vector<double>& targetPos, std::vector<double>& targetJoint) {
	targetJoint.clear();
	KDL::JntArray q_init(initialJoint.size());
	for (int i = 0; i < initialJoint.size(); i++) {
		q_init(i) = initialJoint.at(i);
	}
	KDL::Frame targetFrame(KDL::Rotation::RPY(targetPos[3], targetPos[4], targetPos[5]),KDL::Vector(targetPos[0], targetPos[1], targetPos[2]));
	KDL::JntArray q_out(initialJoint.size());
	int retval;
	//COBOT_LOG.notice() << "q_init:( " << q_init(0) <<"," << q_init(1) << "," << q_init(2) << "," << q_init(3) << "," << q_init(4) << "," << q_init(5) << ")";
	//m_ik_solver->display_information = true;
	retval = m_ik_solver->CartToJnt(q_init, targetFrame, q_out);

	double x, y, z;
	targetFrame.M.GetRPY(x, y, z);

	switch (retval) {
	case 0:
		targetJoint.clear();
		for (int i = 0; i < q_out.data.size(); i++) {
			targetJoint.push_back(q_out(i));
		}
		break;
	case -1:
		COBOT_LOG.error() << "Kinamatic Solver:the gradient of $ E $ towards the joints is to small.";
		break;
	case -2:
		COBOT_LOG.error() << "Kinamatic Solver:joint position increments are to small.";
		break;
	case -3:
		COBOT_LOG.error() << "Kinamatic Solver:number of iterations is exceeded.";
		break;
	}
	//if (retval != 0) {
	//	std::cout << "---------Inverse Kinematic Solver failed ----------------------------" << endl;
	//	COBOT_LOG.error() << "pos " << targetFrame << endl;
	//	std::cout << "reached pos " << m_ik_solver->T_base_head << endl;
	//	std::cout << "TF from pos to head \n" << targetFrame.Inverse()*m_ik_solver->T_base_head << endl;
	//	std::cout << "gradient " << m_ik_solver->grad.transpose() << endl;
	//	std::cout << "q_out " << q_out.data.transpose() / M_PI*180.0 << endl;
	//	std::cout << "q_init " << q_init.data.transpose() / M_PI*180.0 << endl;
	//	std::cout << "return value " << retval << endl;
	//	std::cout << "last #iter " << m_ik_solver->lastNrOfIter << endl;
	//	std::cout << "last diff  " << m_ik_solver->lastDifference << endl;
	//}
	return retval;
}

int KinematicSolver::jntToCart(const std::vector<double>& targetJoint, std::vector<double>& targetPos) {
	KDL::JntArray targetJnt(targetJoint.size());
	for (int i = 0; i < targetJoint.size(); i++) {
		targetJnt(i) = targetJoint.at(i);
	}
	KDL::Frame targetFrame;
	int retval;
	retval=m_fk_solver->JntToCart(targetJnt, targetFrame);
	double x, y, z;
	targetFrame.M.GetRPY(x, y, z);
	targetPos.push_back(targetFrame.p.x());
	targetPos.push_back(targetFrame.p.y());
	targetPos.push_back(targetFrame.p.z());
	targetPos.push_back(x);
	targetPos.push_back(y);
	targetPos.push_back(z);
	return retval;

}
int KinematicSolver::jntToCart(const Eigen::VectorXd& targetJoint, Eigen::Affine3d& targetPos){
	KDL::JntArray targetJnt(targetJoint.size());
	targetJnt.data = targetJoint;
	KDL::Frame targetFrame;
	int retval;
	retval=m_fk_solver->JntToCart(targetJnt, targetFrame);
	tf::transformKDLToEigen(targetFrame, targetPos);
	return retval;
}
int KinematicSolver::cartToJnt(const Eigen::VectorXd& initialJoint, const Eigen::Affine3d& targetPos, Eigen::VectorXd& targetJoint){
	if (initialJoint.size() != targetJoint.size()) {
		COBOT_LOG.error() << "Current joint size is not equal to target joint size!";
		return false;
	}
	KDL::JntArray q_init;
	q_init.data = initialJoint;
	KDL::Frame targetFrame;
	tf::transformEigenToKDL(targetPos, targetFrame);
	KDL::JntArray q_out(targetJoint.size());
	int retval;
	retval=m_ik_solver->CartToJnt(q_init, targetFrame, q_out);
	switch (retval) {
	case 0:
		targetJoint=q_out.data;
		break;
	case -1:
		COBOT_LOG.error() << "Kinamatic Solver:the gradient of $ E $ towards the joints is to small.";
		break;
	case -2:
		COBOT_LOG.error() << "Kinamatic Solver:joint position increments are to small.";
		break;
	case -3:
		COBOT_LOG.error() << "Kinamatic Solver:number of iterations is exceeded.";
		break;
	}
	//if (retval != 0) {
	//	std::cout << "---------Inverse Kinematic Solver failed ----------------------------" << endl;
	//	COBOT_LOG.error() << "pos " << targetFrame << endl;
	//	std::cout << "reached pos " << m_ik_solver->T_base_head << endl;
	//	std::cout << "TF from pos to head \n" << targetFrame.Inverse()*m_ik_solver->T_base_head << endl;
	//	std::cout << "gradient " << m_ik_solver->grad.transpose() << endl;
	//	std::cout << "q_out " << q_out.data.transpose() / M_PI*180.0 << endl;
	//	std::cout << "q_init " << q_init.data.transpose() / M_PI*180.0 << endl;
	//	std::cout << "return value " << retval << endl;
	//	std::cout << "last #iter " << m_ik_solver->lastNrOfIter << endl;
	//	std::cout << "last diff  " << m_ik_solver->lastDifference << endl;
	//}
	return retval;
}
bool KinematicSolver::setup(const QString& configFilePath) {
	//auto a = FileFinder::find(configFilePath.toStdString);
    //load model from json file.
    //TODO 后期需要json文件的有效性验证功能。
	COBOT_LOG.info() << "configFilePath" <<configFilePath.toStdString();
    QJsonObject json;
    if (loadJson(json, configFilePath)) {	
		m_robot_chain.addSegment(Segment(Joint(Joint::None),
			Frame::DH(0.0, 0.0, 0, 0.0)));
        foreach (const QJsonValue & value, json["param"].toArray()) {
            QJsonObject segmentObj = value.toObject();
				double a = segmentObj["dh"].toObject()["a"].toDouble();
				double alpha = segmentObj["dh"].toObject()["alpha"].toDouble();
				double d = segmentObj["dh"].toObject()["d"].toDouble();
				double theta = segmentObj["dh"].toObject()["theta"].toDouble();
				m_robot_chain.addSegment(Segment(Joint(Joint::RotZ),
					Frame::DH(a, alpha, d, theta)));
				JointLimits jointLimits;
				jointLimits.lower = segmentObj["limits"].toObject()["lower"].toDouble();
				jointLimits.upper = segmentObj["limits"].toObject()["upper"].toDouble();
				jointLimits.effort = segmentObj["limits"].toObject()["effort"].toDouble();
				jointLimits.velocity = segmentObj["limits"].toObject()["velocity"].toDouble();
				m_robot_joint_limits.push_back(jointLimits);
        }
		m_fk_solver=new KDL::ChainFkSolverPos_recursive(m_robot_chain);
		m_ik_solver=new KDL::ChainIkSolverPos_LMA(m_robot_chain);
        return true;
    }
    return false;
}
//KDL::Chain KinematicSolver::UR3Chain() {
//	double a[6] = { 0, -0.24365, -0.21325, 0, 0, 0 };
//
//	double d[6] = { 0.1519, 0, 0, 0.11235, 0.08535, 0.0819 };
//
//	double alpha[6] = { 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 };
//
//	double theta[6] = { 0, 0, 0, 0, 0, 0 };
//
//	//double mass[6] = { 2, 3.42, 1.26, 0.8, 0.8, 0.35 };
//
//	//center_of_mass = [[0,-0.02, 0], [0.13, 0, 0.1157], [0.05, 0, 0.0238], [0, 0, 0.01], [0, 0, 0.01], [0, 0, -0.02]]
//
//	KDL::Chain ur3;
//	//joint 0
//	ur3.addSegment(Segment(Joint(Joint::None),
//		Frame::DH(0.0, 0.0, 0, 0.0)
//	));
//	//joint 1
//	for (int i = 0; i < 6; i++)
//	{
//		ur3.addSegment(Segment(Joint(Joint::RotZ),
//			//Frame::DH_Craig1989(a[i], alpha[i], d[i], theta[i])
//			Frame::DH(a[i], alpha[i], d[i], theta[i])
//		));
//	}
//	//joint 7
//	//ur3.addSegment(Segment(Joint(Joint::None),
//	//	Frame::Identity()
//	//));
//	return ur3;
//}
