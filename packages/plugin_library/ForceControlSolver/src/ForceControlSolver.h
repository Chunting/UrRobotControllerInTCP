//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCECONTROLSOLVER_H
#define PROJECT_FORCECONTROLSOLVER_H

#include <mutex>
#include <cobotsys_abstract_force_sensor.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class ForceControlSolver : public QObject, public AbstractObject, public ForceSensorStreamObserver, public ArmRobotRealTimeStatusObserver {
	Q_OBJECT
public:
	ForceControlSolver();
	virtual ~ForceControlSolver();

	virtual bool setup(const QString& configFilePath);

	virtual void onForceSensorDataStreamUpdate(const forcesensor::Wrench& wrench);

public:
	virtual void onArmRobotConnect();
	virtual void onArmRobotDisconnect();
	virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

protected:
	bool m_bcontrol;

};


#endif //PROJECT_FORCECONTROLSOLVER_H
