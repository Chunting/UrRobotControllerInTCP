//
// Created by longhuicai on 2017-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H
#define PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H

#include <mutex>
#include <thread>
#include <QObject>
#include <QHostAddress>
#include <QUdpSocket>
#include <QString>
#include <QTimer>
#include <cobotsys_data_types.h>
#include <condition_variable>

class OptoforceEthernetUDPDriver : public QObject {
	Q_OBJECT
public:

#define OPTOFORCE_UDP_PORT  49152
#define	DEFAULT_FORCE_DIV	10000.0  // Default divide value
#define	DEFAULT_TORQUE_DIV	100000.0 // Default divide value

	OptoforceEthernetUDPDriver(std::condition_variable& rt_msg_cond, QString ip, int hz=125, QObject* parent = nullptr);
	~OptoforceEthernetUDPDriver();

	void startDriver();
	void stopDriver();

	cobotsys::forcesensor::Wrench getState();

	void onConnect();
	void onDisconnect();
	void onReadyRead();

	void setFrequency(int hz) { m_frequency = hz; }
	int getFrequency() { return m_frequency; }
	
	void doUnzero();
	void doZero();

protected:
	void initWrenchData();
	void startStreaming();
	bool waitForNewData();
	void doConnectHost(int delayMSec = 0);

	void recvThreadFunc();
Q_SIGNALS:
	void sensorconnected();
	void sensordisconnected();
protected:
	std::mutex m_mutex;
	std::thread m_thread;//recieve data stream thread

	bool m_isRecievingData;
	bool m_isThreadRunning;
	bool m_isReadyRead;

	QUdpSocket* m_socket;
	QString m_ipAddress;
	quint16 m_port;
	bool m_isConnected;
	int m_reconnectDelay;
	std::condition_variable* m_pMsgCond; //Signals that new vars are available

	int m_frequency;
	unsigned int m_filter;
	int m_packetCount;

	double m_forceScale;//! Scaling factor for converting raw force values from device into Newtons
	double m_torqueScale;//! Scaling factor for converting raw torque values into Newton*meters

	cobotsys::forcesensor::Wrench m_newWrench;
	cobotsys::forcesensor::Wrench m_offsetWrench;
};


#endif //PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H
