//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCEGUIDECONTROLLER_H
#define PROJECT_FORCEGUIDECONTROLLER_H

#include <mutex>
#include <cobotsys_abstract_controller.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_force_sensor.h>
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

	void createRobot();

	void startRobot();
	void stopRobot();

public:
	virtual void onArmRobotConnect();
	virtual void onArmRobotDisconnect();
	virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
	bool m_bcontrol;
	int m_joint_num;
	QString m_robotFactory;
	QString m_robotType;
	QString m_robotConfig;
	std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;

	QString m_sensorFactory;
	QString m_sensorType;
	QString m_sensorConfig;
	std::shared_ptr<AbstractForceSensor> m_ptrSensor;

	QString m_solverFactory;
	QString m_solverType;
	QString m_solverConfig;
	std::shared_ptr<AbstractObject> m_ptrSolver;

	std::vector<double> m_deltaValue;
	std::mutex m_mutex;

};


#endif //PROJECT_FORCEGUIDECONTROLLER_H
