//
// Created by longhuicai on 2017-4-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "optoforce_ethernet_udp_driver.h"
#include <cobotsys_logger.h>


OptoforceEthernetUDPDriver::OptoforceEthernetUDPDriver(QString ip, QObject* parent) : QObject(parent) {
	m_ipAddress = ip;
	//default value
	m_port = 49152;
	m_frequency = 125;
	m_socket = new QUdpSocket(this);
	m_isConnected = false;
	m_reconnectDelay = 100;

	connect(m_socket, &QUdpSocket::connected, this, &OptoforceEthernetUDPDriver::onConnect);
	connect(m_socket, &QUdpSocket::disconnected, this, &OptoforceEthernetUDPDriver::onDisconnect);
	connect(m_socket, &QUdpSocket::hostFound, this, &OptoforceEthernetUDPDriver::onHostFound);
	connect(m_socket, static_cast<void (QUdpSocket::*)(QAbstractSocket::SocketError)>(&QUdpSocket::error), this,
		&OptoforceEthernetUDPDriver::onError);
	connect(m_socket, &QUdpSocket::readyRead, this, &OptoforceEthernetUDPDriver::onDataReady);
}

OptoforceEthernetUDPDriver::~OptoforceEthernetUDPDriver() {
}


void OptoforceEthernetUDPDriver::startDriver() {
}

void OptoforceEthernetUDPDriver::stopDriver() {
}

cobotsys::forcesensor::Wrench OptoforceEthernetUDPDriver::getState() {
	cobotsys::forcesensor::Wrench data;
	return data;
}

void OptoforceEthernetUDPDriver::onConnect() {
	m_isConnected = true;
	COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "Connected";

	processConnect();
}

void OptoforceEthernetUDPDriver::onDisconnect() {
	auto last_connection_status = m_isConnected;
	m_isConnected = false;
	COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "Disconnected";

	if (last_connection_status)
		processDisconnect();

	doConnectHost(m_reconnectDelay);
}

void OptoforceEthernetUDPDriver::onHostFound() {
	//COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "Found Server: " << _socket->peerName();
}

void OptoforceEthernetUDPDriver::onError(QAbstractSocket::SocketError error) {
	//COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << _socket->errorString();

	doConnectHost(m_reconnectDelay);
}

void OptoforceEthernetUDPDriver::onDataReady() {
	auto ba = m_socket->readAll();
	if (ba.size()) {
		processData(ba);
	}
}

void OptoforceEthernetUDPDriver::processData(const QByteArray& ba) {
	//COBOT_LOG.info() << "RECV [" << std::setw(5) << ba.size() << "] " << ba.constData();
}


void OptoforceEthernetUDPDriver::doConnectHost(int delayMSec) {
	if (m_isConnected)
		return;

	auto doCONNECT = [=]() {
		m_socket->connectToHost(m_ipAddress, m_port, QIODevice::ReadWrite);
	};

	if (delayMSec > 0)
		QTimer::singleShot(delayMSec, doCONNECT);
	else
		doCONNECT();
}

void OptoforceEthernetUDPDriver::processConnect() {
}

void OptoforceEthernetUDPDriver::processDisconnect() {
}

