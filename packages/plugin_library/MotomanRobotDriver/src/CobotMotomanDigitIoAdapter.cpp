//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "CobotMotomanDigitIoAdapter.h"

CobotMotomanDigitIoAdapter::CobotMotomanDigitIoAdapter() {
    m_UDPCommCtrl = nullptr;
    m_isInput = false;
    m_isOutput = false;
}

CobotMotomanDigitIoAdapter::~CobotMotomanDigitIoAdapter() {
}

bool CobotMotomanDigitIoAdapter::setup(const QString& configFilePath) {
    return true;
}

void CobotMotomanDigitIoAdapter::setIo(DigitIoPorts ioPorts, DigitIoStatus ioStatus) {
    if (m_isOutput) {
        if (ioPorts & DigitIoPort::Port_0) setDigitOut(0, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_1) setDigitOut(1, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_2) setDigitOut(2, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_3) setDigitOut(3, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_4) setDigitOut(4, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_5) setDigitOut(5, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_6) setDigitOut(6, ioStatus == DigitIoStatus::Set);
        if (ioPorts & DigitIoPort::Port_7) setDigitOut(7, ioStatus == DigitIoStatus::Set);
    }
}

DigitIoStatus CobotMotomanDigitIoAdapter::getIoStatus(DigitIoPort ioPort) {
    if (m_isInput) {
        if (m_inputIoStatus & (int) ioPort) {
            return DigitIoStatus::Set;
        }
    }
    return DigitIoStatus::Reset;
}

bool CobotMotomanDigitIoAdapter::isDigitInput() const {
    return m_isInput;
}

bool CobotMotomanDigitIoAdapter::isDigitOutput() const {
    return m_isOutput;
}

void CobotMotomanDigitIoAdapter::debugIoStatus() {
    if (isDigitInput() && m_debugIoLastStatus != m_inputIoStatus) {
        COBOT_LOG.message("Input") << std::hex << setw(8) << m_inputIoStatus;
        m_debugIoLastStatus = m_inputIoStatus;
    }
    if (isDigitOutput() && m_debugIoLastStatus != m_outputIoStatus) {
        COBOT_LOG.message("Output") << std::hex << setw(8) << m_outputIoStatus;
        m_debugIoLastStatus = m_outputIoStatus;
    }
}

void CobotMotomanDigitIoAdapter::setDigitOut(int portIndex, bool b) {
    char buf[256] = {0};
    if (portIndex < 8) {
        sprintf(buf, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
                portIndex, b ? "True" : "False");
        if (m_UDPCommCtrl) {
            m_UDPCommCtrl->addCommandToQueue(buf);
        }
    }
}

bool CobotMotomanDigitIoAdapter::setToolVoltage(double v) {
    char buf[256];
    int voltage = (int) v;
    sprintf(buf, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", voltage);
    if (m_UDPCommCtrl) {
        m_UDPCommCtrl->addCommandToQueue(buf);
        return true;
    }
    return false;
}

void CobotMotomanDigitIoAdapter::setMotomanUDPCtrl(CobotMotomanUDPCommCtrl* realTimeCommCtrl) {
    m_UDPCommCtrl = realTimeCommCtrl;
    if (m_UDPCommCtrl == nullptr) {
        m_inputIoStatus = 0;
        m_outputIoStatus = 0;
    }

}

bool CobotMotomanDigitIoAdapter::isOpened() const {
    return (m_UDPCommCtrl != nullptr);
}

