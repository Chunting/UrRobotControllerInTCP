//
// Created by longhuicai on 17-4-10.
// Version 1.0.0 finished by longhuicai on 2017-4-18
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "OptoForceSensor.h"
#include <QtCore/QJsonObject>
#include <extra2.h>
#include <algorithm>

OptoForceSensor::OptoForceSensor() :
	QObject(nullptr),
	m_upd_driver(nullptr),
	m_isWatcherRunning(false),
	m_isStarted(false)
{
	m_upd_driver = new OptoforceEthernetUDPDriver(m_rt_msg_cond);
	if (m_upd_driver) {
		connect(m_upd_driver, &OptoforceEthernetUDPDriver::sensorconnected, this, &OptoForceSensor::handleDriverReady);
		connect(m_upd_driver, &OptoforceEthernetUDPDriver::sensordisconnected, this, &OptoForceSensor::handleDriverDisconnect);
		connect(m_upd_driver, &QObject::destroyed, [=](QObject*) { handleDriverDisconnect(); });
	}
}

OptoForceSensor::~OptoForceSensor() {
	if (m_isWatcherRunning) {
		m_isWatcherRunning = false;
		m_rt_msg_cond.notify_all();
		m_thread.join();
	}
}

bool OptoForceSensor::setup(const QString &configFilePath) {
	std::lock_guard<std::mutex> lock_guard(m_mutex);

	auto success = _setup(configFilePath);

	if (!success) {
		m_observers.clear(); // detach all observer
	}
	return success;
}

bool OptoForceSensor::start() {
	//std::lock_guard<std::mutex> lock_guard(m_mutex);

	if (m_isStarted) {
		COBOT_LOG.info() << "Already start, if want restart, stop first";
		return false;
	}

	//set ip and hz
	m_upd_driver->setIp(m_attr_sensor_ip.c_str());
	m_upd_driver->setFrequency(m_attr_sensor_frequency);

	m_upd_driver->startDriver();

	return true;
}

void OptoForceSensor::stop() {
	std::lock_guard<std::mutex> lock_guard(m_mutex);

	if (m_isStarted) {
		m_isStarted = false;
		m_upd_driver->stopDriver();
	}
}

void OptoForceSensor::attach(const std::shared_ptr<ForceSensorStreamObserver>& observer) {
	std::lock_guard<std::mutex> lock_guard(m_mutex);

	for (auto& iter : m_observers) {
		if (iter.get() == observer.get()) {
			return; // Already have attached
		}
	}

	if (observer) {
		m_observers.push_back(observer);
	}
}

void OptoForceSensor::sensorDataWatcher() {
	std::mutex m;
	std::unique_lock<std::mutex> lck(m);

	auto time_cur = std::chrono::high_resolution_clock::now();

	auto pStatus = std::make_shared<Wrench>();

	while (m_isWatcherRunning) {
		m_rt_msg_cond.wait(lck);

		//// 计算时间间隔
		//auto time_rdy = std::chrono::high_resolution_clock::now();
		//std::chrono::duration<double> time_diff = time_rdy - time_cur; // 时间间隙
		//time_cur = time_rdy;
		//COBOT_LOG.info() << "Status Updated: " << time_diff.count();

		// 抓取当前值
		if (m_mutex.try_lock()) {
			if (m_upd_driver) {
				auto state = m_upd_driver->getState();
				pStatus->force = state.force;
				pStatus->torque = state.torque;
			}
			m_mutex.unlock();
		}

		// 通知所有观察者，数据已经更新。
		if (m_isStarted) {
			notify([=](std::shared_ptr<ForceSensorStreamObserver>& observer) {
				observer->onForceSensorDataStreamUpdate(pStatus);
			});
		}
	}
}

bool OptoForceSensor::_setup(const QString& configFilePath) {
	QJsonObject json;
	if (loadJson(json, configFilePath)) {
		//parser json file
		m_attr_sensor_ip = json["sensor_ip"].toString("localhost").toStdString();
		m_attr_sensor_frequency = json["frequency"].toInt(125);
		m_protocol = json["protocol"].toString("UDP").toStdString();
		std::transform(m_protocol.begin(), m_protocol.end(), m_protocol.begin(), ::toupper);//to upper

		m_isWatcherRunning = true;
		m_thread = std::thread(&OptoForceSensor::sensorDataWatcher, this);
		return true;
	}
	return false;
}

void OptoForceSensor::handleDriverReady() {
	//COBOT_LOG.info() << "OptoForce Sensor Connect";
	m_isStarted = true;
	notify([=](std::shared_ptr<ForceSensorStreamObserver>& observer) {
		observer->onForceSensorConnect();
	});
}

void OptoForceSensor::handleDriverDisconnect() {
	//COBOT_LOG.info() << "OptoForce Sensor Disconnect";
	m_isStarted = false;
	notify([=](std::shared_ptr<ForceSensorStreamObserver>& observer) {
		observer->onForceSensorDisconnect();
	});
}

void OptoForceSensor::notify(std::function<void(std::shared_ptr<ForceSensorStreamObserver>& observer)> func) {
	if (func) {
		std::vector<std::shared_ptr<ForceSensorStreamObserver> > observer_tmp;
		if (m_mutex.try_lock()) {
			observer_tmp = m_observers;
			m_mutex.unlock();
		}

		for (auto& observer : observer_tmp) {
			func(observer);
		}
	}
}
