//
// Created by longhuicai on 17-4-10.
//

#include "ForceGuideController.h"
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include <thread>

ForceGuideController::ForceGuideController() : QObject(nullptr) {

}

ForceGuideController::~ForceGuideController() {

}

bool ForceGuideController::setup(const QString &configFilePath) {
	m_bcontrolStart = false;
	m_bRobotConnect = false;
	m_bSensorConnect = false;

	bool ret = false;
	QJsonObject json;
	if (loadJson(json, configFilePath)) {
		m_joint_num = json["joint_num"].toDouble(6);
		m_curQ.resize(m_joint_num);

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
		createForceSensor();
		createKinematicSolver();

		ret = true;
	}

	m_controlThread = std::thread(&ForceGuideController::guideControlThread, this);

	return ret;
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
			COBOT_LOG.notice() << "Create and setup robot success";
			ret = true;
		}
		else {
			m_ptrRobot.reset();
		}
	}
	return ret;
}

bool ForceGuideController::createForceSensor() {
	if (!GlobalObjectFactory::instance()) return false;

	if (m_sensorConfig.isEmpty()) {
		m_sensorConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
			tr("Get Force Sensor Config JSON file ..."),
			QString(FileFinder::getPreDefPath().c_str()),
			tr("JSON files (*.JSON *.json)"));
		if (m_sensorConfig.isEmpty())
			return false;
	}

	bool ret = false;
	auto obj = GlobalObjectFactory::instance()->createObject(m_sensorFactory, m_sensorType);
	m_ptrSensor = std::dynamic_pointer_cast<AbstractForceSensor>(obj);
	if (m_ptrSensor) {
		auto ob = std::dynamic_pointer_cast<ForceSensorStreamObserver>(shared_from_this());
		m_ptrSensor->attach(ob);
		if (m_ptrSensor->setup(m_sensorConfig)) {
			COBOT_LOG.notice() << "Create and setup force sensor success";
			ret = true;
		}
		else {
			m_ptrSensor.reset();
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

void ForceGuideController::guideControlThread() {
	while (true)
	{
		if (m_bcontrolStart) {
			if (m_bRobotConnect)//&&m_bSensorConnect)
			{
				if (m_ptrKinematicSolver) {
					std::vector<double> pos;
					m_ptrKinematicSolver->jntToCart(m_curQ, pos);

					int dir = 1;
					if (pos[2] < 0.2)
						dir = 1;
					else if (pos[2] > 0.4)
						dir = -1;

					pos[2] += 0.01*(dir);

					std::vector<double> targetQ;
					m_ptrKinematicSolver->cartToJnt(m_curQ, pos, targetQ);

					m_ptrRobot->move(targetQ);

					//sleep for loop
					std::chrono::milliseconds timespan(10); 
					std::this_thread::sleep_for(timespan);

				}
				else {
					COBOT_LOG.error() << "kinematic solver not created!";
				}
			}
			else {
				COBOT_LOG.error() << "robot not connected!";
			}
		}
	}
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

void ForceGuideController::startForceSensor() {
	if (m_ptrSensor) {
		if (m_ptrSensor->start()) {
			COBOT_LOG.info() << "Force Sensor Start Success";
		}
	}
}

void ForceGuideController::stopForceSensor() {
	if (m_ptrSensor) {
		m_ptrSensor->stop();
		COBOT_LOG.info() << "Force Sensor Stopped";
	}
}

bool ForceGuideController::start() {
	//
	startForceSensor();
	return true;
	//test
	std::vector<double> src;
	for (int i = 0; i < m_joint_num; ++i)
		src.push_back(0);
	src[0] = 2.04;
	src[1] = 1.25;
	src[2] = 1.02;
	src[3] = 1.02;
	src[4] = 1.25;
	src[5] = 1.30;
	std::vector<double> pos;
	m_ptrKinematicSolver->jntToCart(src, pos);
	std::vector<double> result;
	m_ptrKinematicSolver->cartToJnt(src, pos, result);

	bool ret = true;
	startRobot();
	m_bcontrolStart = true;
	return ret;
}
void ForceGuideController::pause() {
}

void ForceGuideController::stop() {
	stopForceSensor();
	stopRobot();
	m_bcontrolStart = false;
}

void ForceGuideController::onArmRobotConnect() {
	m_bRobotConnect = true;
	COBOT_LOG.info() << "Robot connect";
}

void ForceGuideController::onArmRobotDisconnect() {
	m_bRobotConnect = false;
	COBOT_LOG.info() << "Robot disconnect";
}

void ForceGuideController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
	if (ptrRobotStatus->q_actual.size() != m_joint_num) {
		COBOT_LOG.error() << "actural joint count not equal with value from config.";
		return;
	}

	m_curQ = ptrRobotStatus->q_actual;

	//std::vector<double> pos;
	//m_ptrKinematicSolver->jntToCart(m_curQ, pos);

	//COBOT_LOG.info() << "Robot state update: "
	//	<< "X," << pos[0] << ";"
	//	<< "Y," << pos[1] << ";"
	//	<< "Z," << pos[2] << ";"
	//	<< "r," << pos[3] << ";"
	//	<< "p," << pos[4] << ";"
	//	<< "y," << pos[5] << ";"
	//	<<".";
}


