//
// Created by longhuicai on 2017-4-14.
// Version 1.0.0 finished by longhuicai on 2017-4-18
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "optoforce_ethernet_udp_driver.h"
#include <cobotsys_logger.h>
#include <chrono>

struct HSURecord  // High-speed UDP record
{
	uint32_t hs_sequence_;
	uint32_t ft_sequence_;
	uint32_t status_;
	int32_t fx_;
	int32_t fy_;
	int32_t fz_;
	int32_t tx_;
	int32_t ty_;
	int32_t tz_;

	enum { HSU_RECORD_SIZE = 36 };
	void unpack(const uint8_t *buffer);
	static uint32_t unpack32(const uint8_t *buffer);
};

uint32_t HSURecord::unpack32(const uint8_t *buffer)
{
	return
		(uint32_t(buffer[0]) << 24) |
		(uint32_t(buffer[1]) << 16) |
		(uint32_t(buffer[2]) << 8) |
		(uint32_t(buffer[3]) << 0);
}

void HSURecord::unpack(const uint8_t *buffer)
{
	hs_sequence_ = unpack32(buffer + 0);
	ft_sequence_ = unpack32(buffer + 4);
	status_ = unpack32(buffer + 8);
	fx_ = unpack32(buffer + 12);
	fy_ = unpack32(buffer + 16);
	fz_ = unpack32(buffer + 20);
	tx_ = unpack32(buffer + 24);
	ty_ = unpack32(buffer + 28);
	tz_ = unpack32(buffer + 32);
}


struct HSUCommand
{
	uint16_t command_header_;
	uint16_t command_;
	uint32_t sample_count_;

	HSUCommand() : command_header_(HEADER)
	{
		// empty
	}

	enum { HEADER = 0x1234 };

	// Possible values for command_
	enum {
		CMD_STOP_STREAMING = 0,
		CMD_START_HIGH_SPEED_STREAMING = 2,
		CMD_SET_SPEED = 0x0082,
		CMD_SET_FILTER = 0x0081

	};

	// Special values for sample count
	enum { INFINITE_SAMPLES = 0 };

	enum { HSU_COMMAND_SIZE = 8 };

	//!Packet structure into buffer for network transport
	//  Buffer should be RDT_COMMAND_SIZE
	void pack(uint8_t *buffer) const;
};

void HSUCommand::pack(uint8_t *buffer) const
{
	// Data is big-endian
	buffer[0] = (command_header_ >> 8) & 0xFF;
	buffer[1] = (command_header_ >> 0) & 0xFF;
	buffer[2] = (command_ >> 8) & 0xFF;
	buffer[3] = (command_ >> 0) & 0xFF;
	buffer[4] = (sample_count_ >> 8) & 0xFF;
	buffer[5] = (sample_count_ >> 0) & 0xFF;
	buffer[6] = (sample_count_ >> 8) & 0xFF;
	buffer[7] = (sample_count_ >> 0) & 0xFF;
}


OptoforceEthernetUDPDriver::OptoforceEthernetUDPDriver(std::condition_variable& rt_msg_cond, QObject* parent) :
	QObject(parent),
	m_isRecievingData(false),
	m_isThreadRunning(false),
	m_isReadyRead(false),
	m_ipAddress("localhost"),
	m_port(OPTOFORCE_UDP_PORT),
	m_isConnected(false),
	m_reconnectDelay(100),
	m_pMsgCond(&rt_msg_cond),
	m_frequency(125),
	m_filter(0),
	m_packetCount(0),
	m_forceScale(1.0),
	m_torqueScale(1.0)
{
	//default value
	initWrenchData();
	doUnzero();

	//socket init
	m_socket = new QUdpSocket(this);

	m_socket->setSocketOption(QAbstractSocket::SocketOption::LowDelayOption, 1);
	connect(m_socket, &QUdpSocket::connected, this, &OptoforceEthernetUDPDriver::onConnect);
	connect(m_socket, &QUdpSocket::disconnected, this, &OptoforceEthernetUDPDriver::onDisconnect);
	connect(m_socket, &QUdpSocket::readyRead, this, &OptoforceEthernetUDPDriver::onReadyRead);

	// Get Force/Torque scale from device webserver
	double counts_per_force = DEFAULT_FORCE_DIV;
	double counts_per_torque = DEFAULT_TORQUE_DIV;

	m_forceScale = 1.0 / counts_per_force;
	m_torqueScale = 1.0 / counts_per_torque;

}

OptoforceEthernetUDPDriver::~OptoforceEthernetUDPDriver() {
	m_isThreadRunning = false;
	m_isRecievingData = false;
	m_isReadyRead = false;
	m_socket->close();
	m_pMsgCond->notify_all();
}

void OptoforceEthernetUDPDriver::initWrenchData() {
	m_newWrench.force.x = 0;
	m_newWrench.force.y = 0;
	m_newWrench.force.z = 0;
	m_newWrench.torque.x = 0;
	m_newWrench.torque.y = 0;
	m_newWrench.torque.z = 0;
}

void OptoforceEthernetUDPDriver::recvThreadFunc() {
	m_isThreadRunning = true;
	HSURecord hsu_record;
	cobotsys::Wrench tmp_data;
	while (m_isThreadRunning) 
	{
		if (!m_isConnected) {
			//COBOT_LOG.notice() << "not connected! please do start connection.";
			continue;
		}
		if (!m_isRecievingData) {
			//COBOT_LOG.notice() << "not recieving data! please do start data streaming.";
			continue;
		}
		if (m_isReadyRead)// m_socket->hasPendingDatagrams())
		{
			char* tmp;
			int len;
			{
				std::lock_guard<std::mutex> lock_guard(m_mutex);
				m_isReadyRead = false;

				tmp = m_datagram.data();
				len = m_datagram.size();
			}

			if (len != HSURecord::HSU_RECORD_SIZE)
			{
				COBOT_LOG.notice() << "Receive size of " << len << " bytes does not match expected size of " << HSURecord::HSU_RECORD_SIZE;
			}
			else
			{
				hsu_record.unpack((uint8_t*)tmp);
				{
					tmp_data.force.x = double(hsu_record.fx_) * m_forceScale;
					tmp_data.force.y = double(hsu_record.fy_) * m_forceScale;
					tmp_data.force.z = double(hsu_record.fz_) * m_forceScale;
					tmp_data.torque.x = double(hsu_record.tx_) * m_torqueScale;
					tmp_data.torque.y = double(hsu_record.ty_) * m_torqueScale;
					tmp_data.torque.z = double(hsu_record.tz_) * m_torqueScale;
					{
						std::lock_guard<std::mutex> lock_guard(m_mutex);
						m_newWrench = tmp_data;
						m_newWrench.force.x -= m_offsetWrench.force.x;
						m_newWrench.force.y -= m_offsetWrench.force.y;
						m_newWrench.force.z -= m_offsetWrench.force.z;
						m_newWrench.torque.x -= m_offsetWrench.torque.x;
						m_newWrench.torque.y -= m_offsetWrench.torque.y;
						m_newWrench.torque.z -= m_offsetWrench.torque.z;
						++m_packetCount;
						m_pMsgCond->notify_all();
					}
					//COBOT_LOG.notice() << " wrench:   force: " <<m_newWrench.force.x<<","<<m_newWrench.force.y<<","<<m_newWrench.force.z<<"\r\n"
					//	<<" \t\t\t torque: " << m_newWrench.torque.x << "," << m_newWrench.torque.y << "," << m_newWrench.torque.z << "\r\n";
				}
			}
		} // end if
	}// end while
}

void OptoforceEthernetUDPDriver::startDriver() {
	//check connect
	if (!m_isConnected) 
		doConnectHost();

	// Start receive thread  
	if (!m_isThreadRunning) {
		m_thread = std::thread(&OptoforceEthernetUDPDriver::recvThreadFunc, this);
		m_packetCount = 0;
	}

	// Since start steaming command is sent with UDP packet,
	// the packet could be lost, retry startup 10 times before giving up
	for (int i = 0; i < 10; ++i)
	{
		startStreaming();
		if (waitForNewData())
			break;
	}

}

void OptoforceEthernetUDPDriver::startStreaming()
{
	//std::lock_guard<std::mutex> lock_guard(m_mutex);
	uint8_t buffer[HSUCommand::HSU_COMMAND_SIZE];
	// Command EtherDAQ to set up its speed
	HSUCommand speed_setup;
	speed_setup.command_ = HSUCommand::CMD_SET_SPEED;
	if (m_frequency == 0) {
		m_frequency = 1;
	}
	speed_setup.sample_count_ = 1000 / m_frequency;
	speed_setup.pack(buffer);
	m_socket->write((char*)buffer, HSUCommand::HSU_COMMAND_SIZE);

	// Command EtherDAQ to set up its filter
	HSUCommand filter_setup;
	filter_setup.command_ = HSUCommand::CMD_SET_FILTER;
	filter_setup.sample_count_ = m_filter;
	filter_setup.pack(buffer);
	m_socket->write((char*)buffer, HSUCommand::HSU_COMMAND_SIZE);

	// Command EtherDAQ to start data transmission
	HSUCommand start_transmission;
	start_transmission.command_ = HSUCommand::CMD_START_HIGH_SPEED_STREAMING;
	start_transmission.sample_count_ = HSUCommand::INFINITE_SAMPLES;

	start_transmission.pack(buffer);
	m_socket->write((char*)buffer, HSUCommand::HSU_COMMAND_SIZE);

	m_isRecievingData = true;
}

void OptoforceEthernetUDPDriver::stopDriver() {
	m_isRecievingData = false;
	m_socket->close();
}

cobotsys::Wrench OptoforceEthernetUDPDriver::getState() {
	cobotsys::Wrench data;
	std::lock_guard<std::mutex> lock_guard(m_mutex);
	data = m_newWrench;
	return data;
}

void OptoforceEthernetUDPDriver::onConnect() {
	m_isConnected = true;
	COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "Connected";
	Q_EMIT sensorconnected();
}

void OptoforceEthernetUDPDriver::onReadyRead() {
	m_isReadyRead = true;
	//COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "onReadyRead................................................";

	m_datagram.resize(m_socket->pendingDatagramSize());
	m_socket->readDatagram(m_datagram.data(), m_datagram.size());
	char* tmp = m_datagram.data();
}

void OptoforceEthernetUDPDriver::onDisconnect() {
	m_isConnected = false;
	m_isRecievingData = false;

	COBOT_LOG.notice() << "OptoforceEthernetUDPDriver: " << "Disconnected";
	Q_EMIT sensordisconnected();

	//no reconnect
	//doConnectHost(m_reconnectDelay);
}

void OptoforceEthernetUDPDriver::doConnectHost(int delayMSec) {
	if (m_isConnected)
		return;

	auto doCONNECT = [=]() {
		//socket bind
		QHostAddress address;
		address.setAddress(m_ipAddress);
		m_socket->bind(address, m_port);
		//socket connect
		m_socket->open(QIODevice::ReadWrite);
		m_socket->connectToHost(m_ipAddress, m_port, QIODevice::ReadWrite);
	};

	if (delayMSec > 0)
		QTimer::singleShot(delayMSec, doCONNECT);
	else
		doCONNECT();
}

void OptoforceEthernetUDPDriver::doUnzero() {
	std::lock_guard<std::mutex> lock_guard(m_mutex);
	m_offsetWrench.force.x = 0.0;
	m_offsetWrench.force.y = 0.0;
	m_offsetWrench.force.z = 0.0;
	m_offsetWrench.torque.x = 0.0;
	m_offsetWrench.torque.y = 0.0;
	m_offsetWrench.torque.z = 0.0;
}

void OptoforceEthernetUDPDriver::doZero() {
	std::lock_guard<std::mutex> lock_guard(m_mutex);
	m_offsetWrench.force.x = m_newWrench.force.x;
	m_offsetWrench.force.y = m_newWrench.force.y;
	m_offsetWrench.force.z = m_newWrench.force.y;
	m_offsetWrench.torque.x = m_newWrench.torque.x;
	m_offsetWrench.torque.y = m_newWrench.torque.y;
	m_offsetWrench.torque.z = m_newWrench.torque.z;
}

bool OptoforceEthernetUDPDriver::waitForNewData()
{
	// Wait upto 100ms for new data
	bool got_new_data = false;
	{
		std::lock_guard<std::mutex> lock_guard(m_mutex);
		unsigned current_packet_count = m_packetCount;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		got_new_data = (m_packetCount != current_packet_count);
	}

	return got_new_data;
}