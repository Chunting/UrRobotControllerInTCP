//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "ForceGuideController.h"
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>
#include <thread>

ForceGuideController::ForceGuideController() : 
	QObject(nullptr),
	m_ptrRobot(nullptr),
	m_ptrSensor(nullptr),
	m_ptrForceControlSolver(nullptr),
	m_ptrKinematicSolver(nullptr),
	m_bcontrolStart(false),
	m_bRobotConnect(false),
	m_bSensorConnect(false)
{

}

ForceGuideController::~ForceGuideController() {

}

bool ForceGuideController::setup(const QString &configFilePath) {
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

		//solver first
		createKinematicSolver();
		createForceControlSolver();

		//robot next
		createRobot();
		createForceSensor();


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
		if (m_ptrForceControlSolver) {
			auto obs = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(m_ptrForceControlSolver);
			m_ptrRobot->attach(obs);
		}
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

bool ForceGuideController::createForceControlSolver() {
	if (!GlobalObjectFactory::instance()) return false;

	if (m_solverConfig.isEmpty()) {
		m_solverConfig = QFileDialog::getOpenFileName(Q_NULLPTR,
			tr("Get Force Control Solver Config JSON file ..."),
			QString(FileFinder::getPreDefPath().c_str()),
			tr("JSON files (*.JSON *.json)"));
		if (m_solverConfig.isEmpty())
			return false;
	}

	bool ret = false;
	auto obj = GlobalObjectFactory::instance()->createObject(m_solverFactory, m_solverType);
	m_ptrForceControlSolver = std::dynamic_pointer_cast<AbstractForceControlSolver>(obj);
	if (m_ptrForceControlSolver) {
		if (m_ptrForceControlSolver->setup(m_solverConfig)) {
			COBOT_LOG.notice() << "Create and setup success";
			ret = true;
			if (m_ptrRobot) {
				auto obs = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(m_ptrForceControlSolver);
				m_ptrRobot->attach(obs);
			}
		}
		else {
			m_ptrForceControlSolver.reset();
		}
	}
	return ret;
}

void ForceGuideController::guideControlThread() {
	while (true)
	{
		if (m_bcontrolStart) {
			if (!m_bRobotConnect) {
				COBOT_LOG.error() << "robot not connected!";
				continue;
			}
			if (!m_bSensorConnect) {
				COBOT_LOG.error() << "force sensor not connected!";
				continue;
			}
			if (m_ptrForceControlSolver)
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
				COBOT_LOG.error() << "Force control solver not created!";
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
	bool ret = true;
	//
	startRobot();
	//
	startForceSensor();
	m_bcontrolStart = true;
	return ret;
}
void ForceGuideController::pause() {
}

void ForceGuideController::stop() {
	m_bcontrolStart = false;
	stopForceSensor();
	stopRobot();
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

void ForceGuideController::onForceSensorConnect() {
    m_bSensorConnect = true;
	COBOT_LOG.info() << "Force sensor connected";
}

void ForceGuideController::onForceSensorDisconnect() {
	m_bSensorConnect = false;
	COBOT_LOG.info() << "Force sensor disconnected";
}

void ForceGuideController::onForceSensorDataStreamUpdate(const std::shared_ptr<forcesensor::Wrench>& ptrWrench) {
	m_wrenchData.force.x = ptrWrench->force.x;
	m_wrenchData.force.y = ptrWrench->force.y;
	m_wrenchData.force.z = ptrWrench->force.z;
	m_wrenchData.torque.x = ptrWrench->torque.x;
	m_wrenchData.torque.y = ptrWrench->torque.y;
	m_wrenchData.torque.z = ptrWrench->torque.z;
	//COBOT_LOG.notice() << " wrench:   force: " << m_wrenchData.force.x<<","<< m_wrenchData.force.y<<","<< m_wrenchData.force.z<<"\r\n"
	//	<<" \t\t\t torque: " << m_wrenchData.torque.x << "," << m_wrenchData.torque.y << "," << m_wrenchData.torque.z << "\r\n";
}


