//
// Created by longhuicai on 2017-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H
#define PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H

#include <QObject>
#include <QHostAddress>
#include <QUdpSocket>
#include <QString>
#include <QTimer>
#include <cobotsys_data_types.h>

class OptoforceEthernetUDPDriver : public QObject {
	Q_OBJECT
public:
	OptoforceEthernetUDPDriver(QString ip, QObject* parent = nullptr);
	~OptoforceEthernetUDPDriver();

	void startDriver();
	void stopDriver();

	void onConnect();
	void onDisconnect();

	void setFrequency(int hz) { m_frequency = hz; }
	int getFrequency() { return m_frequency; }

	cobotsys::forcesensor::Wrench getState();
protected:
	void onHostFound();
	void onError(QAbstractSocket::SocketError error);

	void onDataReady();

	void doConnectHost(int delayMSec = 0);

protected:
	virtual void processData(const QByteArray& ba);
	virtual void processConnect();
	virtual void processDisconnect();


protected:
	QUdpSocket* m_socket;
	QString m_ipAddress;
	quint16 m_port;
	int m_frequency;
	bool m_isConnected;
	int m_reconnectDelay;
};


#endif //PROJECT_OPTOFORCE_ETHERNET_UDP_DRIVER_H
