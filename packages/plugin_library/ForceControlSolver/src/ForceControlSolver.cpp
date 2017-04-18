//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "ForceControlSolver.h"

ForceControlSolver::ForceControlSolver() : QObject(nullptr) {

}

ForceControlSolver::~ForceControlSolver() {

}

void ForceControlSolver::onForceSensorConnect() {
}

void ForceControlSolver::onForceSensorDisconnect() {
}

void ForceControlSolver::onForceSensorDataStreamUpdate(const std::shared_ptr<forcesensor::Wrench>& ptrWrench) {
	m_wrenchData.force.x = ptrWrench->force.x;
	m_wrenchData.force.y = ptrWrench->force.y;
	m_wrenchData.force.z = ptrWrench->force.z;
	m_wrenchData.torque.x = ptrWrench->torque.x;
	m_wrenchData.torque.y = ptrWrench->torque.y;
	m_wrenchData.torque.z = ptrWrench->torque.z;
	//COBOT_LOG.notice() << " wrench:   force: " << m_wrenchData.force.x<<","<< m_wrenchData.force.y<<","<< m_wrenchData.force.z<<"\r\n"
	//	<<" \t\t\t torque: " << m_wrenchData.torque.x << "," << m_wrenchData.torque.y << "," << m_wrenchData.torque.z << "\r\n";
}

bool ForceControlSolver::setup(const QString& configFilePath) {
	return true;
}

void ForceControlSolver::onArmRobotConnect() {
}

void ForceControlSolver::onArmRobotDisconnect() {
}

void ForceControlSolver::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
	m_curQ = ptrRobotStatus->q_actual;
}

int ForceControlSolver::solve(std::vector<double>& targetQ) {
	return 0;
}

int ForceControlSolver::solve(const forcesensor::Wrench& wrench, const std::vector<double>& currentQ, std::vector<double>& targetQ) {
	return 0;
}

