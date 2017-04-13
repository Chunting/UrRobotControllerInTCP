//
// Created by lhc on 17-4-10.
//

#include "ForceGuideController.h"
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

ForceGuideController::ForceGuideController() {

}

ForceGuideController::~ForceGuideController() {

}

bool ForceGuideController::setup(const QString &configFilePath) {
	QJsonObject json;
	if (loadJson(json, configFilePath)) {
		m_joint_num = json["joint_num"].toDouble(6);
		m_robotFactory = json["robot_object"].toObject()["factory"].toString();
		m_robotType = json["robot_object"].toObject()["type"].toString();
		m_robotConfig = json["robot_object"].toObject()["config"].toString();
		if (m_robotFactory.isEmpty() || m_robotType.isEmpty() ) {
			return false;
		}
		
		m_sensorFactory = json["sensor_object"].toObject()["factory"].toString();
		m_sensorType = json["sensor_object"].toObject()["type"].toString();
		m_sensorConfig = json["sensor_object"].toObject()["config"].toString();
		if (m_sensorFactory.isEmpty() || m_sensorType.isEmpty()) {
			return false;
		}

		m_solverFactory = json["solver_object"].toObject()["factory"].toString();
		m_solverType = json["solver_object"].toObject()["type"].toString();
		m_solverConfig = json["solver_object"].toObject()["config"].toString();
		if (m_solverFactory.isEmpty() || m_solverType.isEmpty()) {
			return false;
		}

		m_kinematicFactory = json["kinematic_object"].toObject()["factory"].toString();
		m_kinematicType = json["kinematic_object"].toObject()["type"].toString();
		m_kinematicConfig = json["kinematic_object"].toObject()["config"].toString();
		if (m_kinematicFactory.isEmpty() || m_kinematicType.isEmpty()) {
			return false;
		}

		int jmin = json["joint_min"].toInt(-180);
		int jmax = json["joint_max"].toInt(180);

		createRobot();
//		createForceSensor();
		createKinematicSolver();

		return true;
	}
	return false;
}


bool ForceGuideController::createRobot() {
	if (!GlobalObjectFactory::instance()) return false;

	if (m_robotConfig.isEmpty()) {
		m_robotConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
			tr("Get Robot Config JSON file ..."),
			QString(FileFinder::getPreDefPath().c_str()),
			tr("JSON files (*.JSON *.json)"));
		if (m_robotConfig.isEmpty())
			return false;
	}

	bool ret = false;
	auto obj = GlobalObjectFactory::instance()->createObject(m_robotFactory, m_robotType);
	m_ptrRobot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
	if (m_ptrRobot) {
		auto ob = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this());
		m_ptrRobot->attach(ob);
		if (m_ptrRobot->setup(m_robotConfig)) {
			COBOT_LOG.notice() << "Create and setup success";
			ret = true;
		}
		else {
			m_ptrRobot.reset();
		}
	}
	return ret;
}

bool ForceGuideController::createKinematicSolver() {
	if (!GlobalObjectFactory::instance()) return false;

	if (m_kinematicConfig.isEmpty()) {
		m_kinematicConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
			tr("Get Kinematic Solver Config JSON file ..."),
			QString(FileFinder::getPreDefPath().c_str()),
			tr("JSON files (*.JSON *.json)"));
		if (m_kinematicConfig.isEmpty())
			return false;
	}

	bool ret = false;
	auto obj = GlobalObjectFactory::instance()->createObject(m_kinematicFactory, m_kinematicType);
	m_ptrKinematicSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
	if (m_ptrKinematicSolver) {
		if (m_ptrKinematicSolver->setup(m_kinematicConfig)) {
			COBOT_LOG.notice() << "Create and setup success";
			ret = true;
		}
		else {
			m_ptrKinematicSolver.reset();
		}
	}
	return ret;
}


void ForceGuideController::startRobot() {
	if (m_ptrRobot) {
		if (m_ptrRobot->start()) {
			COBOT_LOG.info() << "Robot Start Success";
		}
	}
}

void ForceGuideController::stopRobot() {
	if (m_ptrRobot) {
		m_ptrRobot->stop();
		COBOT_LOG.info() << "Robot stopped";
	}
}


bool ForceGuideController::start() {
	bool ret = true;
	startRobot();
	return ret;
}
void ForceGuideController::pause() {
}

void ForceGuideController::stop() {
	stopRobot();
}

void ForceGuideController::onArmRobotConnect() {
	COBOT_LOG.info() << "Robot connect";
}

void ForceGuideController::onArmRobotDisconnect() {
	COBOT_LOG.info() << "Robot disconnect";
}

void ForceGuideController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {


	COBOT_LOG.info() << "Robot state update";
}


