//
// Created by longhuicai on 17-4-10.
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
}

bool ForceControlSolver::setup(const QString& configFilePath) {
	return true;
}

void ForceControlSolver::onArmRobotConnect() {
}

void ForceControlSolver::onArmRobotDisconnect() {
}

void ForceControlSolver::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
}

