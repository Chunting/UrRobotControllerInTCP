//
// Created by 潘绪洋 on 17-3-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTURFIRMWAREQUERYER_H
#define PROJECT_COBOTURFIRMWAREQUERYER_H

#include "../URDriver/robot_state.h"
#include <QObject>
#include <memory>
#include <QTcpSocket>
#include <cobotsys_logger.h>
#include "CobotUr.h"
#include <cobotsys_qt.h>

class CobotUrFirmwareQueryer : public QObject {
Q_OBJECT

public:
    QString m_host;
public:
    CobotUrFirmwareQueryer(const QString& robotIp = "localhost"){
        m_host = robotIp;
    }

    ~CobotUrFirmwareQueryer(){
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

        if (tcpSocket->waitForConnected(CobotUr::MAX_SOCKET_WAIT_)) {
            if (tcpSocket->waitForReadyRead(CobotUr::MAX_SOCKET_WAIT_)) {
                auto ba = tcpSocket->read(512);
                if (robotState) {
                    robotState->unpack((uint8_t*) ba.constData(), ba.size());
                    COBOT_LOG.info() << "Firmware version detected: " << robotState->getVersion();
                }
                tcpSocket->close();
                return robotState;
            }
        }
        COBOT_LOG.error() << "CobotUrFirmwareQueryer: " << tcpSocket->errorString();
        return nullptr;
    }
};


#endif //PROJECT_COBOTURFIRMWAREQUERYER_H
