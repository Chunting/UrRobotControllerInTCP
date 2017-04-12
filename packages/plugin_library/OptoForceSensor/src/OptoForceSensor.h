//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_OPTOFORCESENSOR_H
#define PROJECT_OPTOFORCESENSOR_H

#include <mutex>
#include <cobotsys_abstract_force_sensor.h>
#include <QObject>
#include <QString>

using namespace cobotsys;

class OptoForceSensor : public QObject, public AbstractForceSensor {
	Q_OBJECT
public:
	OptoForceSensor();
	virtual ~OptoForceSensor();

	virtual bool setup(const QString& configFilePath);
	virtual bool open(int deviceId = 0);
	virtual void close();
	virtual void attach(const shared_ptr<ForceSensorStreamObserver>& observer);
protected:
	bool m_bcontrol;

};


#endif //PROJECT_OPTOFORCESENSOR_H
