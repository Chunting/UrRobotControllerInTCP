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
#include "frames_io.hpp"
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
	KDL::Frame targetFrame = stdVectortoFrame(targetPos);
	KDL::JntArray q_out(initialJoint.size());
	int retval;
	//COBOT_LOG.notice() << "q_init:( " << q_init(0) <<"," << q_init(1) << "," << q_init(2) << "," << q_init(3) << "," << q_init(4) << "," << q_init(5) << ")";
	//m_ik_solver->display_information = true;
	retval = m_ik_solver->CartToJnt(q_init, targetFrame, q_out);

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
	default:
		COBOT_LOG.error() << "Kinamatic Solver:  unknown error.";
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

int KinematicSolver::cartToJnt(const Eigen::VectorXd& initialJoint, const std::vector<double>& targetPos, std::vector<double>& targetJoint) {
	targetJoint.clear();
	KDL::JntArray q_init(initialJoint.size());
	q_init.data = initialJoint;
	KDL::Frame targetFrame = stdVectortoFrame(targetPos);
	KDL::JntArray q_out(initialJoint.size());
	int retval;
	//COBOT_LOG.notice() << "q_init:( " << q_init(0) <<"," << q_init(1) << "," << q_init(2) << "," << q_init(3) << "," << q_init(4) << "," << q_init(5) << ")";
	//m_ik_solver->display_information = true;
	retval = m_ik_solver->CartToJnt(q_init, targetFrame, q_out);

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
	default:
		COBOT_LOG.error() << "Kinamatic Solver:  unknown error.";
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

int KinematicSolver::cartToJnt(const Eigen::VectorXd& initialJoint, const Eigen::Affine3d& targetPos, Eigen::VectorXd& targetJoint) {
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
	retval = m_ik_solver->CartToJnt(q_init, targetFrame, q_out);
	switch (retval) {
	case 0:
		targetJoint = q_out.data;
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
	default:
		COBOT_LOG.error() << "Kinamatic Solver:  unknown error.";
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
	targetPos = frameToStdVector(targetFrame);
	return retval;
}

int KinematicSolver::jntToCart(const Eigen::VectorXd& targetJoint, Eigen::Affine3d& targetPos) {
	KDL::JntArray targetJnt(targetJoint.size());
	targetJnt.data = targetJoint;
	KDL::Frame targetFrame;
	int retval;
	retval = m_fk_solver->JntToCart(targetJnt, targetFrame);
	tf::transformKDLToEigen(targetFrame, targetPos);
	return retval;
}

std::vector<double> KinematicSolver::frameToStdVector(KDL::Frame frame) {
	std::vector<double> vec;
	double x, y, z;
	frame.M.GetRPY(x, y, z);
	vec.push_back(frame.p.x());
	vec.push_back(frame.p.y());
	vec.push_back(frame.p.z());
	vec.push_back(x);
	vec.push_back(y);
	vec.push_back(z);
	return vec;
}
KDL::Frame KinematicSolver::stdVectortoFrame(std::vector<double> vec) {
	if (vec.size()!=6) {
		COBOT_LOG.error() << "std vector size is not 6";
		return KDL::Frame();
	}
	KDL::Frame frame(KDL::Rotation::RPY(vec[3], vec[4], vec[5]), 
		KDL::Vector(vec[0], vec[1], vec[2]));
	return frame;
}

int KinematicSolver::vector_WorldToEE(const Eigen::VectorXd& jointArray, const Eigen::Vector3d& vector_world, Eigen::Vector3d& vector_ee) {
	KDL::JntArray targetJnt(jointArray.size());
	targetJnt.data = jointArray;
	KDL::Frame frame_world2ee;
	int retval;
	retval = m_fk_solver->JntToCart(targetJnt, frame_world2ee);
	KDL::Vector vect_w(vector_world.x(), vector_world.y(), vector_world.z());
	KDL::Vector vect_ee = frame_world2ee.Inverse().M*vect_w;
	vector_ee = Eigen::Vector3d(vect_ee.x(), vect_ee.y(), vect_ee.z());
	return retval;
}
int KinematicSolver::vector_WorldToEE(const std::vector<double>& jointArray, const Eigen::Vector3d& vector_world, Eigen::Vector3d& vector_ee) {
	KDL::JntArray targetJnt(jointArray.size());
	for (int i = 0; i < jointArray.size(); i++) {
		targetJnt(i) = jointArray[i];
	}
	KDL::Frame frame_world2ee;
	int retval;
	retval = m_fk_solver->JntToCart(targetJnt, frame_world2ee);
	KDL::Vector vect_w(vector_world.x(), vector_world.y(), vector_world.z());
	KDL::Vector vect_ee = frame_world2ee.Inverse().M*vect_w;
	vector_ee = Eigen::Vector3d(vect_ee.x(), vect_ee.y(), vect_ee.z());
	return retval;
}

int KinematicSolver::pose_EEToWorld(const Eigen::VectorXd& jointArray, const std::vector<double>& pose_ee, std::vector<double>& pose_world) {
	KDL::JntArray targetJnt(jointArray.size());
	targetJnt.data = jointArray;
	KDL::Frame frame_ee=stdVectortoFrame(pose_ee);
	KDL::Frame frame_world2ee;
	int retval;
	retval = m_fk_solver->JntToCart(targetJnt, frame_world2ee);
	KDL::Frame frame_world = frame_world2ee*frame_ee;
	pose_world.clear();
	pose_world = frameToStdVector(frame_world);
	return retval;
}
int KinematicSolver::pose_EEToWorld(const std::vector<double>& jointArray, const std::vector<double>& pose_ee, std::vector<double>& pose_world) {
	KDL::JntArray targetJnt(jointArray.size());
	for (int i = 0; i < jointArray.size(); i++) {
		targetJnt(i)= jointArray[i];
	}
	KDL::Frame frame_ee = stdVectortoFrame(pose_ee);
	KDL::Frame frame_world2ee;
	int retval;
	retval = m_fk_solver->JntToCart(targetJnt, frame_world2ee);
	KDL::Frame frame_world = frame_world2ee*frame_ee;
	pose_world.clear();
	pose_world = frameToStdVector(frame_world);
	return retval;
}

bool KinematicSolver::setup(const QString& configFilePath) {
	//auto a = FileFinder::find(configFilePath.toStdString);
    //load model from json file.
    //TODO 后期需要json文件的有效性验证功能。
	//TODO 做base frame到 world frame的segment。
	//TODO 做joint6到机器人末端执行器的segmen。
	//COBOT_LOG.info() << "configFilePath" <<configFilePath.toStdString();
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
		//world_base
		QJsonArray data;
		double pt[3], rpy[3];
		data = json["world_base"].toObject()["xyz"].toArray();
		for (int i = 0; i < 3; i++) {
			pt[i] = data.at(i).toDouble(0);
		}
		data = json["world_base"].toObject()["RPY"].toArray();
		for (int i = 0; i < 3; i++) {
			rpy[i] = data.at(i).toDouble(0);
		}
		KDL::Frame wb;
		wb.p = KDL::Vector(pt[0],pt[1],pt[2]);
		wb.M = KDL::Rotation::RPY(rpy[0], rpy[1], rpy[2]);
		m_robot_chain.addSegment(Segment(Joint(Joint::None),
			wb));	
	//	m_robot_chain.addSegment(Segment(Joint(Joint::None),
	//		Frame::DH(0.0, 0.0, 0, 0.0)));
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
		m_robot_chain.addSegment(Segment(Joint(Joint::None),
            Frame::DH(0.0,0.0,0.01,-M_PI_2)));//to modify end frame.
		m_fk_solver=new KDL::ChainFkSolverPos_recursive(m_robot_chain);
		m_ik_solver=new KDL::ChainIkSolverPos_LMA(m_robot_chain);

		//for (int i = 0; i < m_robot_chain.getNrOfSegments();++i) {
		//	KDL::Segment seg =  m_robot_chain.getSegment(i);
		//	std::string name = seg.getName();
		//	KDL::Frame pos = seg.getFrameToTip();
		//	KDL::Joint jnt = seg.getJoint();
		//	std::string type = jnt.getTypeName();
		//	KDL::Vector axis = jnt.JointAxis();
		//	KDL::Vector org = jnt.JointOrigin();
		//}
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
