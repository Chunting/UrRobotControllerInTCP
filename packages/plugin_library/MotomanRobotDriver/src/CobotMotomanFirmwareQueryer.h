//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_FIRMWARE_QUERYER_H
#define COBOT_MOTOMAN_FIRMWARE_QUERYER_H

#include <QObject>
#include <memory>
#include <QTcpSocket>
#include <cobotsys_logger.h>
#include "CobotMotoman.h"
#include <cobotsys.h>
#include "motoman_robot_state.h"
class CobotMotomanFirmwareQueryer : public QObject {
Q_OBJECT
//TODO DONE
public:
    QString m_host;
public:
    CobotMotomanFirmwareQueryer(const QString& robotIp = "localhost"){
        m_host = robotIp;
    }

    ~CobotMotomanFirmwareQueryer(){
    }

    /**
     * 获取UR的版本号，如果连接失败，那么返回一个空的指针。
     * @param robotState
     * @return
     */
    std::shared_ptr<RobotState> getVersion(std::shared_ptr<RobotState> robotState){
        QTcpSocket* tcpSocket = new QTcpSocket(this);
        tcpSocket->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        tcpSocket->connectToHost(m_host, 30001);

        if (tcpSocket->waitForConnected(CobotMotoman::MAX_SOCKET_WAIT_)) {
            if (tcpSocket->waitForReadyRead(CobotMotoman::MAX_SOCKET_WAIT_)) {
                auto ba = tcpSocket->read(512);
                if (robotState) {
                    robotState->unpack((uint8_t*) ba.constData(), ba.size());
                    COBOT_LOG.info() << "Firmware version detected: " << robotState->getVersion();
                }
                tcpSocket->close();
                return robotState;
            }
        }
        COBOT_LOG.error() << "CobotMotomanFirmwareQueryer: " << tcpSocket->errorString();
        return nullptr;
    }
};


#endif //COBOT_MOTOMAN_FIRMWARE_QUERYER_H
