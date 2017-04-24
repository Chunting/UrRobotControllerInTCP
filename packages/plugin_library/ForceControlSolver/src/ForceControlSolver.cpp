//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "ForceControlSolver.h"
#include <QtCore/QJsonObject>
#include <QtCore/qjsonarray.h>
#include <extra2.h>

ForceControlSolver::ForceControlSolver() : QObject(nullptr) {
	for (int i = 0; i < 3; i++)
	{
		m_gravity[i] = 0;
		m_gcenter[i] = 0;
	}
	for (int i = 0; i < 6; i++)
	{
		m_biasRepair[i] = 0;
		m_forceEE[i] = 0;
		m_gravityEE[i] = 0;
		m_offsetEE[i] = 0;
	}

	m_jntSize = 6;
	m_posEfFactor = 0.1;
	m_posEf.init(m_jntSize, m_posEfFactor);

	//construct ForceControllerClass
	ForceControllerClass();
}

ForceControlSolver::~ForceControlSolver() {

}

void ForceControlSolver::onForceSensorConnect() {
}

void ForceControlSolver::onForceSensorDisconnect() {
}

void ForceControlSolver::onForceSensorDataStreamUpdate(const std::shared_ptr<cobotsys::Wrench>& ptrWrench) {
	m_wrenchData.force.x = ptrWrench->force.x;
	m_wrenchData.force.y = ptrWrench->force.y;
	m_wrenchData.force.z = ptrWrench->force.z;
	m_wrenchData.torque.x = ptrWrench->torque.x;
	m_wrenchData.torque.y = ptrWrench->torque.y;
	m_wrenchData.torque.z = ptrWrench->torque.z;
	//COBOT_LOG.notice() << " wrench:   force: " << m_wrenchData.force.x<<","<< m_wrenchData.force.y<<","<< m_wrenchData.force.z<<"\r\n"
	//	<<" \t\t\t torque: " << m_wrenchData.torque.x << "," << m_wrenchData.torque.y << "," << m_wrenchData.torque.z << "\r\n";

	calcForceEE();

	//
	step(m_forceEE, m_gravityEE, m_offsetEE);
	/*COBOT_LOG.notice() << " offset:   transition: " << m_offsetEE [0]<<","<< m_offsetEE [1]<<","<< m_offsetEE [2]<<"\r\n"
		<<" \t\t\t rotation: " << m_offsetEE[3] << "," << m_offsetEE[4] << "," << m_offsetEE[5] << "\r\n";*/
}

void ::ForceControlSolver::calcForceEE() {
	m_forceEE[0] = m_wrenchData.force.x;
	m_forceEE[1] = m_wrenchData.force.y;
	m_forceEE[2] = m_wrenchData.force.z;
	m_forceEE[3] = m_wrenchData.torque.x;
	m_forceEE[4] = m_wrenchData.torque.y;
	m_forceEE[5] = m_wrenchData.torque.z;
	//bias repair
	for (size_t i = 0; i < 6; i++)
	{
		m_forceEE[i] -= m_biasRepair[i];
	}
	// transform from sensor to ee
	Eigen::Vector3d f;
	f(0) = m_forceEE[0];
	f(1) = m_forceEE[1];
	f(2) = m_forceEE[2];
	Eigen::Vector3d t;
	t(0) = m_forceEE[3];
	t(1) = m_forceEE[4];
	t(2) = m_forceEE[5];
	Eigen::Matrix3d ft_ee;
	ft_ee << 0, 1, 0, -1, 0, 0, 0, 0, 1;//todo should be defined in the config, different robot or EE could be different values. 
	Eigen::Vector3d r;
	r = ft_ee*f;
	m_forceEE[0] = r(0);
	m_forceEE[1] = r(1);
	m_forceEE[2] = r(2);
	r = ft_ee*t;
	m_forceEE[3] = r(0);
	m_forceEE[4] = r(1);
	m_forceEE[5] = r(2);

}

bool ForceControlSolver::setup(const QString& configFilePath) {
	//init ForceControllerClass
	this->initialize();
	//
	QJsonObject json;
	if (loadJson(json, configFilePath)) {
		//parser json file
		QJsonArray data;
		//todo gravity data should be calculate by robot pose move
		//for now, just input the config values
		//gravity
		data = json["gravity_repair"].toObject()["gravity"].toArray();
		for (int i = 0; i < 3; i++) {
			m_gravity[i] = data.at(i).toDouble();
		}
		data = json["gravity_repair"].toObject()["center"].toArray();
		for (int i = 0; i < 3; i++) {
			m_gcenter[i] = data.at(i).toDouble();
		}
		//bias_repair
		data = json["bias_repair"].toArray();
		for (int i = 0; i < 6; i++) {
			m_biasRepair[i] = data.at(i).toDouble();
		}
		//filter_param
		data = json["filter_param"].toObject()["den"].toArray();
		for (int i = 0; i < 3; i++) {
			m_param.filter_den[i] = data.at(i).toDouble();
		}
		data = json["filter_param"].toObject()["num"].toArray();
		for (int i = 0; i < 3; i++) {
			m_param.filter_num[i] = data.at(i).toDouble();
		}
		//filter_param
		data = json["pid_factor"].toObject()["P"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.P[i] = data.at(i).toDouble();
		}
		data = json["pid_factor"].toObject()["I"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.I[i] = data.at(i).toDouble();
		}
		data = json["pid_factor"].toObject()["D"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.D[i] = data.at(i).toDouble();
		}
		data = json["pid_factor"].toObject()["N"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.N[i] = data.at(i).toDouble();
		}
		//dead_zone
		data = json["dead_zone"].toObject()["start"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.dead_zone_start[i] = data.at(i).toDouble();
		}
		data = json["dead_zone"].toObject()["end"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.dead_zone_end[i] = data.at(i).toDouble();
		}
		//saturation_limit
		data = json["saturation_limit"].toObject()["lower"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.saturation_lower_limit[i] = data.at(i).toDouble();
		}
		data = json["saturation_limit"].toObject()["upper"].toArray();
		for (int i = 0; i < 6; i++) {
			m_param.saturation_upper_limit[i] = data.at(i).toDouble();
		}

		//set
		setBlockParameters(&m_param);
		return true;
	}
	return false;
}

void ForceControlSolver::onArmRobotConnect() {
}

void ForceControlSolver::onArmRobotDisconnect() {
}

void ForceControlSolver::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
	m_curQ = ptrRobotStatus->q_actual;
	calGravityEE();
}

void ForceControlSolver::calGravityEE() {
	//force ee repair
	Eigen::Vector3d vgravity(m_gravity);
	//vgravity.x() = m_gravity[0];
	//vgravity.y() = m_gravity[1];
	//vgravity.z() = m_gravity[2];

	Eigen::Vector3d eeGravity;
	if (m_ptrKinematicSolver) {
		m_ptrKinematicSolver->vector_WorldToEE(m_curQ, vgravity, eeGravity);
	}
	else {
		COBOT_LOG.error() << "kinematic solver not created!";
	}
	for (size_t i = 0; i < 3; i++) {
		m_gravityEE[i] = eeGravity[i];
	}
	// torque ee repair
	m_gravityEE[3] = m_gcenter[0] * (eeGravity.y() + eeGravity.z());
	m_gravityEE[4] = m_gcenter[1] * (eeGravity.z() + eeGravity.x());
	m_gravityEE[5] = m_gcenter[2] * (eeGravity.x() + eeGravity.y());

}

int ForceControlSolver::solve(std::vector<double>& offset) {
	offset.clear();
	std::vector<double> src;
	for (int i = 0; i < 6; i++)
	{
		src.push_back(m_offsetEE[i]);
	}

	m_posEf.doFilter(src, offset);
	return 0;
}

int ForceControlSolver::solve(const cobotsys::Wrench& wrench, const std::vector<double>& currentQ, std::vector<double>& offset) {
	double force[6];
	double gravity[6];
	double roffset[6];
	force[0] = wrench.force.x;
	force[1] = wrench.force.y;
	force[2] = wrench.force.z;
	force[3] = wrench.torque.x;
	force[4] = wrench.torque.y;
	force[5] = wrench.torque.z;
	//bias repair
	for (size_t i = 0; i < 6; i++)
	{
		force[i] -= m_biasRepair[i];
	}
	//todo transform from sensor to ee

	// gravity 
	//force ee repair
	Eigen::Vector3d vgravity(m_gravity);
	//vgravity.x() = m_gravity[0];
	//vgravity.y() = m_gravity[1];
	//vgravity.z() = m_gravity[2];

	Eigen::Vector3d eeGravity;
	if (m_ptrKinematicSolver) {
		m_ptrKinematicSolver->vector_WorldToEE(m_curQ, vgravity, eeGravity);
	}
	else {
		COBOT_LOG.error() << "kinematic solver not created!";
	}
	for (size_t i = 0; i < 3; i++) {
		m_gravityEE[i] = eeGravity[i];
	}
	// torque ee repair
	m_gravityEE[3] = m_gcenter[0] * (eeGravity.y() + eeGravity.z());
	m_gravityEE[4] = m_gcenter[1] * (eeGravity.z() + eeGravity.x());
	m_gravityEE[5] = m_gcenter[2] * (eeGravity.x() + eeGravity.y());

	//solve
	step(force, gravity, roffset);

	//output
	offset.clear();
	for (int i = 0; i < 6; i++)
	{
		offset.push_back(roffset[i]);
	}
	return 0;
}

