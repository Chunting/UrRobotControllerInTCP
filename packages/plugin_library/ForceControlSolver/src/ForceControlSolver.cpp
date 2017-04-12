//
// Created by lhc on 17-4-10.
//

#include "ForceControlSolver.h"

ForceControlSolver::ForceControlSolver() {

}

ForceControlSolver::~ForceControlSolver() {

}

void ForceControlSolver::onForceSensorDataStreamUpdate(const forcesensor::Wrench& wrench) {
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

