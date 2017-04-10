//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCECONTROLSOLVER_H
#define PROJECT_FORCECONTROLSOLVER_H

#include <mutex>
#include <cobotsys_abstract_force_sensor.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class ForceControlSolver : public QObject, public ForceSensorStreamObserver {
	Q_OBJECT
public:
	ForceControlSolver();
	virtual ~ForceControlSolver();

	virtual void onForceSensorDataStreamUpdate(const forcesensor::Wrench& wrench);
protected:
	bool m_bcontrol;

};


#endif //PROJECT_FORCECONTROLSOLVER_H
