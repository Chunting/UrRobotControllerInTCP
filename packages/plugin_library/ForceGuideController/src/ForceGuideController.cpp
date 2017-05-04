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
	m_bSensorConnect(false),
	m_posReady(false),
	m_sensorReady(false),
	m_setVoltage(false)
{
	//m_firstMove = true;
    m_exit = false;
}

ForceGuideController::~ForceGuideController() {
    m_exit = true;
	if (m_controlThread.joinable()){
		m_controlThread.join();
	}

}

bool ForceGuideController::setup(const QString &configFilePath) {
	bool ret = false;
	QJsonObject json;
	if (loadJson(json, configFilePath)) {
		m_joint_num = json["joint_num"].toDouble(6);
		m_curQ.resize(m_joint_num);

		m_robotFactory = json["robot_object"].toObject()["factory"].toString("URRealTimeDriverFactory, Ver 1.0");
		m_robotType = json["robot_object"].toObject()["type"].toString("URRealTimeDriver");
		m_robotConfig = json["robot_object"].toObject()["config"].toString();
		if (m_robotFactory.isEmpty() || m_robotType.isEmpty() ) {
			return false;
		}
				
		m_sensorFactory = json["sensor_object"].toObject()["factory"].toString("OptoForceSensorFactory, Ver 1.0");
		m_sensorType = json["sensor_object"].toObject()["type"].toString("OptoForceSensor");
		m_sensorConfig = json["sensor_object"].toObject()["config"].toString();
		if (m_sensorFactory.isEmpty() || m_sensorType.isEmpty()) {
			return false;
		}

		m_solverFactory = json["solver_object"].toObject()["factory"].toString("ForceControlSolverFactory, Ver 1.0");
		m_solverType = json["solver_object"].toObject()["type"].toString("ForceControlSolver");
		m_solverConfig = json["solver_object"].toObject()["config"].toString(m_sensorConfig);
		if (m_solverFactory.isEmpty() || m_solverType.isEmpty()) {
			return false;
		}

		m_kinematicFactory = json["kinematic_object"].toObject()["factory"].toString("KinematicSolverFactory, Ver 1.0");
		m_kinematicType = json["kinematic_object"].toObject()["type"].toString("KinematicSolver");
		m_kinematicConfig = json["kinematic_object"].toObject()["config"].toString(m_robotConfig);
		if (m_kinematicFactory.isEmpty() || m_kinematicType.isEmpty()) {
			return false;
		}

		int jmin = json["joint_min"].toInt(-180);
		int jmax = json["joint_max"].toInt(180);

		//kinematic solver first
		createKinematicSolver();
		//force control solver second
		createForceControlSolver();
		//set kinematic solver
		if (m_ptrForceControlSolver&&m_ptrKinematicSolver) {
			m_ptrForceControlSolver->setKinematicSolver(m_ptrKinematicSolver);
		}

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
			COBOT_LOG.notice() << "Robot setup success";
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
		if (m_ptrForceControlSolver) {
			auto obs = std::dynamic_pointer_cast<ForceSensorStreamObserver>(m_ptrForceControlSolver);
			m_ptrSensor->attach(obs);
		}
		if (m_ptrSensor->setup(m_sensorConfig)) {
			COBOT_LOG.notice() << "Force sensor setup success";
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
#ifdef DEBUG
			COBOT_LOG.notice() << "KIN solver success";
#endif // DEBUG

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
#ifdef DEBUG
			COBOT_LOG.notice() << "FC solver success";
#endif // DEBUG

			ret = true;
			if (m_ptrRobot) {
				auto obs = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(m_ptrForceControlSolver);
				m_ptrRobot->attach(obs);
			}
			if (m_ptrSensor) {
				auto obs = std::dynamic_pointer_cast<ForceSensorStreamObserver>(m_ptrForceControlSolver);
				m_ptrSensor->attach(obs);
			}
		}
		else {
			m_ptrForceControlSolver.reset();
		}
	}
	return ret;
}

void ForceGuideController::guideControlThread() {
	auto time_cur = std::chrono::high_resolution_clock::now();
	int nc = 0;
    static bool robotConnectCheck = true;
    static bool posReadyCheck = true;
    static bool sensorConnectCheck = true;
    static bool sensorReadyCheck = true;
	static bool buttonStateCheck = true;
    static bool controlSolverCheck = true;
    static bool kinematicSolverCheck = true;
	while (!m_exit)
	{
		std::chrono::duration<double> dur(0.008);
		std::this_thread::sleep_until(time_cur + dur);
		auto time_rdy = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> time_diff = time_rdy - time_cur; // 时间间隙
		time_cur = time_rdy;
		nc++;
		//COBOT_LOG.info() << "control period: " << time_diff.count();

		if (m_bcontrolStart) {
          //check robot connection
          if (!m_bRobotConnect) {
            if (robotConnectCheck) {
              COBOT_LOG.error() << "robot not connected!";
              robotConnectCheck = false;
            }
            continue;
          } else {
            if (!robotConnectCheck) {
              COBOT_LOG.notice() << "robot connected!";
              robotConnectCheck = true;
            }
          }

          //check joint data ready
          if (!m_posReady) {
            if (posReadyCheck) {
              COBOT_LOG.error() << "joint data not ready!";
              posReadyCheck = false;
            }
            continue;
          } else {
            if (!posReadyCheck) {
              COBOT_LOG.notice() << "joint data ready!";
              posReadyCheck = true;
            }
          }

          //setToolVoltage
          auto ioStatus = m_ptrRobot->getDigitIoDriver(1);
          if (!m_setVoltage) {
            //setToolVoltage
            ioStatus->setToolVoltage(12.0);
            m_setVoltage = true;
          }

          //check force sensor connection
          if (!m_bSensorConnect) {
            if (sensorConnectCheck) {
              COBOT_LOG.error() << "force sensor not connected!";
              sensorConnectCheck = false;
            }
            continue;
          } else {
            if (!sensorConnectCheck) {
              COBOT_LOG.notice() << "force sensor connected!";
              sensorConnectCheck = true;
            }
          }

          //check sensor data ready
          if (!m_sensorReady) {
            if (sensorReadyCheck) {
              COBOT_LOG.error() << "force sensor data not ready!";
              sensorReadyCheck = false;
            }
            continue;
          } else {
            if (!sensorReadyCheck) {
              COBOT_LOG.notice() << "force sensor data ready!";
              sensorReadyCheck = true;
            }
          }

          //check control solver ready
          if (!m_ptrForceControlSolver) {
            if (controlSolverCheck) {
              COBOT_LOG.error() << "force control solver not created!";
              controlSolverCheck = false;
            }
            continue;
          } else {
            if (!controlSolverCheck) {
              COBOT_LOG.notice() << "force control solver created!";
              controlSolverCheck = true;
            }
          }

          //check control solver ready
          if (!m_ptrKinematicSolver) {
            if (kinematicSolverCheck) {
              COBOT_LOG.error() << "kinematic solver not created!";
              kinematicSolverCheck = false;
            }
            continue;
          } else {
            if (!kinematicSolverCheck) {
              COBOT_LOG.notice() << "kinematic solver created!";
              kinematicSolverCheck = true;
            }
          }

          m_mutex.lock();
          std::vector<double> curQ = m_curQ;
          m_mutex.unlock();

          //check button state
          if (ioStatus->getIoStatus(DigitIoPort::Port_Ur_Tool_In_0) == DigitIoStatus::Reset) {
            if (buttonStateCheck) {
              COBOT_LOG.warning() << "io button released!";
              buttonStateCheck = false;

              //stop robot motion
              m_ptrRobot->move(curQ);
            }
            continue;
          }
          else {
            if (!buttonStateCheck) {
              COBOT_LOG.notice() << "io button pressed!";
              buttonStateCheck = true;
            }
          }

          //do force guider working
          std::vector<double> offset;
          m_ptrForceControlSolver->solve(offset);

          std::vector<double> offset_ee;
          for (int i = 0; i < 6; i++) {
            offset_ee.push_back(offset[i]);
          }

          std::vector<double> pos;
          m_ptrKinematicSolver->pose_EEToWorld(curQ, offset_ee, pos);
          std::vector<double> targetQ;
          if (m_ptrKinematicSolver->cartToJnt(curQ, pos, targetQ) == 0) {
            m_ptrRobot->move(targetQ);
            //if (m_firstMove) {
            //	m_firstMove = false;
            //	COBOT_LOG.notice() << "first move: " << targetQ[0] << ", " << targetQ[1] << ", " << targetQ[2] << ", " << targetQ[3] << ", " << targetQ[4] << ", " << targetQ[5];
            //}
          }

 		}
	}
}

void ForceGuideController::startRobot() {
	if (m_ptrRobot) {
		if (m_ptrRobot->start()) {
			COBOT_LOG.notice() << "Robot Start Success";
		}
	}
}

void ForceGuideController::stopRobot() {
	if (m_ptrRobot) {
		m_ptrRobot->stop();
		COBOT_LOG.notice() << "Robot stopped";
	}
}

void ForceGuideController::startForceSensor() {
	if (m_ptrSensor) {
		if (m_ptrSensor->start()) {
			COBOT_LOG.notice() << "Force Sensor Start Success";
		}
	}
}

void ForceGuideController::stopForceSensor() {
	if (m_ptrSensor) {
		m_ptrSensor->stop();
		COBOT_LOG.notice() << "Force Sensor Stopped";
	}
}

bool ForceGuideController::start() {
	bool ret = true;

	//
	startForceSensor();

	//
	startRobot();

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
	COBOT_LOG.notice() << "Robot connect";
}

void ForceGuideController::onArmRobotDisconnect() {
	m_bRobotConnect = false;
	m_posReady = false;
	m_setVoltage = false;
	COBOT_LOG.notice() << "Robot disconnect";
}

void ForceGuideController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
	if (ptrRobotStatus->q_actual.size() != m_joint_num) {
		COBOT_LOG.error() << "actural joint count not equal with value from config.";
		return;
	}

	m_mutex.lock();
	m_curQ = ptrRobotStatus->q_actual;
	m_posReady = true;
	m_mutex.unlock();

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

	//auto ioStatus = m_ptrRobot->getDigitIoDriver(1);
	//if (ioStatus->getIoStatus(DigitIoPort::Port_Ur_Tool_In_0) == DigitIoStatus::Reset) {
	//	COBOT_LOG.error() << "false";
	//}
	//else
	//{
	//	COBOT_LOG.error() << "true";
	//}
}

void ForceGuideController::onForceSensorConnect() {
    m_bSensorConnect = true;
	COBOT_LOG.notice() << "Force sensor connected";
}

void ForceGuideController::onForceSensorDisconnect() {
	m_bSensorConnect = false;
	m_sensorReady = false;
	COBOT_LOG.notice() << "Force sensor disconnected";
}

void ForceGuideController::onForceSensorDataStreamUpdate(const std::shared_ptr<cobotsys::Wrench>& ptrWrench) {
	m_mutex.lock();
	m_wrenchData.force.x = ptrWrench->force.x;
	m_wrenchData.force.y = ptrWrench->force.y;
	m_wrenchData.force.z = ptrWrench->force.z;
	m_wrenchData.torque.x = ptrWrench->torque.x;
	m_wrenchData.torque.y = ptrWrench->torque.y;
	m_wrenchData.torque.z = ptrWrench->torque.z;
	m_sensorReady = true;
	m_mutex.unlock();
	//COBOT_LOG.notice() << " wrench:   force: " << m_wrenchData.force.x<<","<< m_wrenchData.force.y<<","<< m_wrenchData.force.z<<"\r\n"
	//	<<" \t\t\t torque: " << m_wrenchData.torque.x << "," << m_wrenchData.torque.y << "," << m_wrenchData.torque.z << "\r\n";
}


