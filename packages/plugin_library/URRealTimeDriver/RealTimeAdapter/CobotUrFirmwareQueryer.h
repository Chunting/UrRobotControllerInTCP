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

    std::shared_ptr<RobotState> getVersion(std::shared_ptr<RobotState> robotState){
        QTcpSocket* m_priSOCK = new QTcpSocket(this);
        m_priSOCK->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        m_priSOCK->connectToHost(m_host, 30001);

        if (m_priSOCK->waitForConnected()) {
            if (m_priSOCK->waitForReadyRead()) {
                auto ba = m_priSOCK->read(512);
                if (robotState) {
                    robotState->unpack((uint8_t*) ba.constData(), ba.size());
                    COBOT_LOG.info() << "Firmware version detected: " << robotState->getVersion();
                }
                m_priSOCK->close();
                return robotState;
            }
        }
        return nullptr;
    }
};


#endif //PROJECT_COBOTURFIRMWAREQUERYER_H
