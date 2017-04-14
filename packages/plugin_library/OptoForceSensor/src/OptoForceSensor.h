//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_OPTOFORCESENSOR_H
#define PROJECT_OPTOFORCESENSOR_H

#include <mutex>
#include <thread>
#include <cobotsys_abstract_force_sensor.h>
#include <QObject>
#include <QString>
#include "../driver/optoforce_ethernet_udp_driver.h"

using namespace cobotsys;

class OptoForceSensor : public QObject, public AbstractForceSensor {
	Q_OBJECT
public:
	OptoForceSensor();
	virtual ~OptoForceSensor();

	virtual bool setup(const QString& configFilePath);
	virtual bool start();
	virtual void stop();
	virtual void attach(const shared_ptr<ForceSensorStreamObserver>& observer);
protected:
	void sensorDataWatcher();

	bool _setup(const QString& configFilePath);

	void handleDriverReady();
	void handleDriverDisconnect();
	void notify(std::function<void(std::shared_ptr<ForceSensorStreamObserver>& observer)> func);

protected:
	std::mutex m_mutex;
	std::thread m_thread;
	bool m_isWatcherRunning;
	bool m_isStarted;

	std::string m_attr_sensor_ip;
	std::string m_protocol;
	int m_attr_sensor_frequency;

	std::vector<std::shared_ptr<ForceSensorStreamObserver> > m_observers;

	OptoforceEthernetUDPDriver *m_upd_driver;

};


#endif //PROJECT_OPTOFORCESENSOR_H
