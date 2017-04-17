//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCEGUIDECONTROLLER_H
#define PROJECT_FORCEGUIDECONTROLLER_H

#include <mutex>
#include <thread>
#include <cobotsys_abstract_controller.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_force_sensor.h>
#include <cobotsys_abstract_kinematic_solver.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class ForceGuideController : public QObject, public AbstractController, public ArmRobotRealTimeStatusObserver {
	Q_OBJECT
public:
	ForceGuideController();
	virtual ~ForceGuideController();

	virtual bool setup(const QString& configFilePath);
	virtual bool start();
	virtual void pause();
	virtual void stop();

protected:
	bool createRobot();
	bool createForceSensor();
	bool createKinematicSolver();

	void startRobot();
	void stopRobot();

	void startForceSensor();
	void stopForceSensor();

	void guideControlThread();

public:
	virtual void onArmRobotConnect();
	virtual void onArmRobotDisconnect();
	virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
	bool m_bcontrolStart;

	int m_joint_num;
	std::vector<double> m_curQ;

	QString m_robotFactory;
	QString m_robotType;
	QString m_robotConfig;
	std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
	bool m_bRobotConnect;

	QString m_sensorFactory;
	QString m_sensorType;
	QString m_sensorConfig;
	std::shared_ptr<AbstractForceSensor> m_ptrSensor;
	bool m_bSensorConnect;

	QString m_solverFactory;
	QString m_solverType;
	QString m_solverConfig;
	std::shared_ptr<AbstractObject> m_ptrSolver;

	QString m_kinematicFactory;
	QString m_kinematicType;
	QString m_kinematicConfig;
	std::shared_ptr<AbstractKinematicSolver> m_ptrKinematicSolver;

	std::vector<double> m_deltaValue;
	std::mutex m_mutex;
	std::thread m_controlThread;

};


#endif //PROJECT_FORCEGUIDECONTROLLER_H
