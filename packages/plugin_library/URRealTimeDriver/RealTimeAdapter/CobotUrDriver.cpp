//
// Created by 潘绪洋 on 17-3-29.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_file_finder.h>
#include <fstream>
#include "CobotUrDriver.h"

CobotUrDriver::CobotUrDriver(std::shared_ptr<std::condition_variable>& msg,
                             const QString& robotAddr, QObject* parent) : QObject(parent) {
    m_urCommCtrl = new CobotUrCommCtrl(robotAddr, this);
    m_urRealTimeCommCtrl = new CobotUrRealTimeCommCtrl(msg, robotAddr, this);

    connect(m_urCommCtrl->ur, &CobotUrComm::connected, this, &CobotUrDriver::handleCommConnected);
    connect(m_urRealTimeCommCtrl->ur, &CobotUrRealTimeComm::connected, this, &CobotUrDriver::handleRTCommConnected);

    connect(m_urCommCtrl->ur, &CobotUrComm::disconnected, this, &CobotUrDriver::handleDisconnected);
    connect(m_urRealTimeCommCtrl->ur, &CobotUrRealTimeComm::disconnected, this, &CobotUrDriver::handleDisconnected);

    connect(m_urRealTimeCommCtrl->ur, &CobotUrRealTimeComm::realTimeProgConnected,
            this, &CobotUrDriver::handleRTProgConnect);
    connect(m_urRealTimeCommCtrl->ur, &CobotUrRealTimeComm::realTimeProgDisconnect,
            this, &CobotUrDriver::handleRTProgDisconnect);

    connect(m_urCommCtrl->ur, &CobotUrComm::connectFail, this, &CobotUrDriver::handleDisconnected);
    connect(m_urRealTimeCommCtrl->ur, &CobotUrRealTimeComm::connectFail, this, &CobotUrDriver::handleDisconnected);


    m_noDisconnectedAccept = false;

    servoj_gain_ = 300;
    servoj_lookahead_time_ = 0.05;
    servoj_time_ = 0.016;

    m_connectTime = 0;
    m_isConnected = false;
}

CobotUrDriver::~CobotUrDriver() {
    stopDriver();
    INFO_DESTRUCTOR(this);
}

void CobotUrDriver::handleCommConnected() {
    auto ver = m_urCommCtrl->ur->getRobotState()->getVersion();
    COBOT_LOG.notice() << "CobotUrDriver::handleCommConnected: version: " << ver;
    m_urRealTimeCommCtrl->ur->getRobotState()->setVersion(m_urCommCtrl->ur->getRobotState()->getVersion());
    m_urRealTimeCommCtrl->startComm();
    m_connectTime++;
    if (m_connectTime >= 2) {
        onConnectSuccess();
    }
}

void CobotUrDriver::handleRTCommConnected() {
    m_connectTime++;
    if (m_connectTime >= 2) {
        onConnectSuccess();
    }
}

void CobotUrDriver::startDriver() {
    m_noDisconnectedAccept = true;
    m_disconnectCount = 0;
    m_connectTime = 0;
    m_isConnected = false;
    m_urCommCtrl->startComm();
}

void CobotUrDriver::handleDisconnected() {
    m_connectTime = 0;
    m_isConnected = false;
    m_disconnectCount++;

    if (m_noDisconnectedAccept) {
        Q_EMIT driverStartFailed();
    }
    if (m_disconnectCount >= 2) {
        Q_EMIT driverStopped();
    }
}

void CobotUrDriver::stopDriver() {
    m_noDisconnectedAccept = false;
    m_urRealTimeCommCtrl->requireStopServoj();
}

bool CobotUrDriver::uploadProg() {
    std::string cmd_str;
    char buf[128];
    cmd_str = "def driverProg():\n";

    sprintf(buf, "\tMULT_jointstate = %i\n", m_urRealTimeCommCtrl->ur->MULT_JOINTSTATE_);
    cmd_str += buf;

    cmd_str += "\tkeepalive = 1\n";
    cmd_str += "\tSERVO_IDLE = 0\n";
    cmd_str += "\tSERVO_RUNNING = 1\n";
    cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
    cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
    cmd_str += "\tservo_loop_n = 0\n";
    cmd_str += "\tservo_loop_old = 0\n";
    cmd_str += "\tdef set_servo_setpoint(q):\n";
    cmd_str += "\t\tenter_critical\n";
    cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
    cmd_str += "\t\tcmd_servo_q = q\n";
    cmd_str += "\t\texit_critical\n";
    cmd_str += "\tend\n";
    cmd_str += "\tnum_brake = 0\n";
    cmd_str += "\tbrake_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
    cmd_str += "\tthread servoThread():\n";
    cmd_str += "\t\tstate = SERVO_IDLE\n";
    cmd_str += "\t\twhile keepalive > 0:\n";
    cmd_str += "\t\t\tenter_critical\n";
    cmd_str += "\t\t\tq = cmd_servo_q\n";
    cmd_str += "\t\t\tbrake_q = cmd_servo_q\n";
    cmd_str += "\t\t\tdo_brake = False\n";
    cmd_str += "\t\t\tif (state == SERVO_RUNNING) and (cmd_servo_state == SERVO_IDLE):\n";
    cmd_str += "\t\t\t\tdo_brake = True\n";
    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tnum_brake = 0\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\t\tstate = cmd_servo_state\n";
    cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
    cmd_str += "\t\t\texit_critical\n";
    cmd_str += "\t\t\tif do_brake:\n";
    cmd_str += "\t\t\t\tservo_loop_n = servo_loop_n + 1\n";
    cmd_str += "\t\t\t\tnum_brake = num_brake + 1\n";
    cmd_str += "\t\t\t\tif num_brake < 5:\n";
    cmd_str += "\t\t\t\t\tservoj(brake_q)\n";
    cmd_str += "\t\t\t\telse:\n";
    cmd_str += "\t\t\t\t\tstopj(1.0)\n";
    cmd_str += "\t\t\t\t\tsync()\n";
    cmd_str += "\t\t\t\tend\n";
    cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";

    if (m_urCommCtrl->ur->getRobotState()->getVersion() >= 3.1)
        sprintf(buf, "\t\t\t\tservoj(q, t=%.4f, lookahead_time=%.4f, gain=%.0f)\n",
                servoj_time_, servoj_lookahead_time_, servoj_gain_);
    else
        sprintf(buf, "\t\t\t\tservoj(q, t=%.4f)\n", servoj_time_);

    cmd_str += buf;

    cmd_str += "\t\t\telse:\n";
    cmd_str += "\t\t\t\tsync()\n";
    cmd_str += "\t\t\tend\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\tstopj(1.0)\n";
    cmd_str += "\tend\n";

    sprintf(buf, "\tsocket_open(\"%s\", %i)\n", ip_addr_.c_str(), m_urRealTimeCommCtrl->ur->REVERSE_PORT_);
    cmd_str += buf;

    cmd_str += "\tthread_servo = run servoThread()\n";

    cmd_str += "\twhile keepalive > 0:\n";
    cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";
    cmd_str += "\t\tif params_mult[0] > 0:\n";
    cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
    cmd_str += "params_mult[2] / MULT_jointstate, ";
    cmd_str += "params_mult[3] / MULT_jointstate, ";
    cmd_str += "params_mult[4] / MULT_jointstate, ";
    cmd_str += "params_mult[5] / MULT_jointstate, ";
    cmd_str += "params_mult[6] / MULT_jointstate]\n";
    cmd_str += "\t\t\tkeepalive = params_mult[7]\n";
    cmd_str += "\t\t\tset_servo_setpoint(q)\n";
    cmd_str += "\t\tend\n";
    cmd_str += "\t\tif servo_loop_old < servo_loop_n:\n";
    cmd_str += "\t\t\tservo_loop_old = servo_loop_n\n";
    cmd_str += "\t\t\tsocket_set_var(\"TCPBrake\", num_brake)\n";
    cmd_str += "\t\t\tsocket_set_var(\"ServoLoop\", servo_loop_n)\n";
    cmd_str += "\t\tend\n";

//    cmd_str += "\t\tif num_brake > 0:\n";
//    cmd_str += "\t\tend\n";

    cmd_str += "\tend\n";
    cmd_str += "\tsleep(.1)\n";
    cmd_str += "\tsocket_close()\n";
    cmd_str += "\tkill thread_servo\n";
    cmd_str += "\tstopj(1.0)\n";
    cmd_str += "end\n";

    auto cmd_file = cobotsys::FileFinder::find("CONFIG/UrDriverScript/UrDriverScript");
    if (cmd_file.size()) {
        std::fstream file_;
        file_.open(cmd_file, std::ios_base::in);
        if (file_.is_open()) {
            file_.seekg(0, std::ios_base::end);
            auto size_ = file_.tellg();
            file_.seekg(0, std::ios_base::beg);
            std::vector<char> buf_((int) size_ + 1, 0);
            file_.read(&buf_[0], size_);
            cmd_str = &buf_[0];

            QString script = cmd_str.c_str();

            if (m_urCommCtrl->ur->getRobotState()->getVersion() >= 3.1)
                sprintf(buf, "t=%.4f, lookahead_time=%.4f, gain=%.0f", servoj_time_, servoj_lookahead_time_,
                        servoj_gain_);
            else
                sprintf(buf, "t=%.4f", servoj_time_);

            script.replace("ARG_HOST_ADDR", ip_addr_.c_str());
            script.replace("ARG_HOST_PORT", QString::number(m_urRealTimeCommCtrl->ur->REVERSE_PORT_));
            script.replace("ARG_SERVOJ_ARGS", buf);
            cmd_str = script.toStdString();
        }
    }


    m_urRealTimeCommCtrl->addCommandToQueue(cmd_str.c_str());
    COBOT_LOG.notice() << "URScript: \n" << cmd_str;
    return true;
}

void CobotUrDriver::setServojTime(double t) {
    if (t > 0.008) {
        servoj_time_ = t;
    } else {
        servoj_time_ = 0.008;
    }
}

void CobotUrDriver::setServojLookahead(double t) {
    if (t > 0.03) {
        if (t < 0.2) {
            servoj_lookahead_time_ = t;
        } else {
            servoj_lookahead_time_ = 0.2;
        }
    } else {
        servoj_lookahead_time_ = 0.03;
    }
}

void CobotUrDriver::setServojGain(double g) {
    if (g > 100) {
        if (g < 2000) {
            servoj_gain_ = g;
        } else {
            servoj_gain_ = 2000;
        }
    } else {
        servoj_gain_ = 100;
    }
}

void CobotUrDriver::onConnectSuccess() {
    m_isConnected = true;
    ip_addr_ = m_urCommCtrl->ur->getLocalIp();
    COBOT_LOG.notice() << "Local Ip: " << ip_addr_;
    COBOT_LOG.notice() << "Version : " << m_urCommCtrl->ur->getRobotState()->getVersion();
    m_urRealTimeCommCtrl->ur->getRobotState()->setVersion(m_urCommCtrl->ur->getRobotState()->getVersion());
    uploadProg();
    Q_EMIT driverStartSuccess();
}

void CobotUrDriver::handleRTProgConnect() {
    COBOT_LOG.info() << "Prog Upload Success";
}

void CobotUrDriver::handleRTProgDisconnect() {
    if (m_isConnected) {
        Q_EMIT driverStopped();
    }
}

void CobotUrDriver::servoj(const std::vector<double>& positions) {
    if (m_urRealTimeCommCtrl) {
        m_urRealTimeCommCtrl->ur->asyncServoj(positions);
    }
}
