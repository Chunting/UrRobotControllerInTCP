//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCECONTROLSOLVER_H
#define PROJECT_FORCECONTROLSOLVER_H

#include <mutex>
#include <cobotsys_abstract_forcecontrol_solver.h>
#include <cobotsys_abstract_force_sensor.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class ForceControlSolver : public QObject, public AbstractForceControlSolver, public ForceSensorStreamObserver, public ArmRobotRealTimeStatusObserver {
	Q_OBJECT
public:
	ForceControlSolver();
	virtual ~ForceControlSolver();

	virtual bool setup(const QString& configFilePath);

	virtual void onForceSensorConnect();
	virtual void onForceSensorDisconnect();
	virtual void onForceSensorDataStreamUpdate(const std::shared_ptr<forcesensor::Wrench>& ptrWrench);

public:
	virtual void onArmRobotConnect();
	virtual void onArmRobotDisconnect();
	virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

public:
	virtual int solve(const forcesensor::Wrench& wrench, const std::vector<double>& currentQ, std::vector<double>& targetQ);
	virtual int solve(std::vector<double>& targetQ);

protected:
	bool m_bcontrol;
	cobotsys::forcesensor::Wrench m_wrenchData;
	std::vector<double> m_curQ;
};


#endif //PROJECT_FORCECONTROLSOLVER_H
