//
// Created by 杨帆 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef MOTOMAN_ROBOT_DRIVER_H
#define MOTOMAN_ROBOT_DRIVER_H

#include "cobotsys_abstract_arm_robot_realtime_driver.h"
#include <QObject>
#include <Eigen/Dense>
#include <QTcpSocket>
#include <QtNetwork>
#include <QDataStream>
#include <QThread>
#include <memory>


QT_BEGIN_NAMESPACE
class QTcpSocket;

class QNetworkSession;

QT_END_NAMESPACE
using namespace cobotsys;
class MotomanRobotObserver {
public:
    virtual void onCallbackPosition(const Eigen::Vector3f& position) = 0;
};

class MotomanRobotDriver : public QObject, public cobotsys::AbstractArmRobotRealTimeDriver {

Q_OBJECT
public:
    MotomanRobotDriver(QObject* parent = 0);

    void setObserver(const std::shared_ptr<MotomanRobotObserver>& observer) {
        m_observer = observer;
    }

    virtual bool setup(const QString& configFilePath);
    /**
     * 这个是一个实时控制的接口，每次调用都会直接反应到当前控制周期内的效果
     * 同一个控制周期内，多次调用，以最后一次调用的值为准。
     * @param q 目标Joint位置
     */
    virtual void move(const std::vector<double>& q);

    /**
     * 获取机器人自带的IO接口控制
     * @param[in] deviceId
     * @return
     */
    virtual std::shared_ptr<AbstractDigitIoDriver> getDigitIoDriver(int deviceId = 0);

    /**
     * 向机器人注册观察者，当机器人事件发生后，会通过观察者接口API来通知所有已注册对象。
     * 顺序通知。
     * 已经实现线程安全
     * @param observer
     */
    virtual void attach(const std::shared_ptr<ArmRobotRealTimeStatusObserver>& observer);

    /**
     * 启动机器人控制。仅仅只是发送命令过程没有问题。实际上需要
     * ArmRobotRealTimeStatusObserver::onArmRobotConnect 函数调用才是
     * 正常的启动了机器人。
     * @note 这个调用完成后，必然会触发观察事件。是否成功启动机器人，以回调事件为准
     * @retval true 机器人启动命令发送成功
     * @retval false 命令发送失败， 具体原因参见log.
     */
    virtual bool start();

    /**
     * 停止实时机器人控制
     */
    virtual void stop();

    /**
     * 返回机器人的实际IP，即 setup() 里设置的IP，可以从这个读取
     * @return 机器人当前的URL地址
     */
    virtual QString getRobotUrl();

    // TODO finish this function
    virtual std::vector<double> getRobotJointQ() { return std::vector<double>(); }

    Eigen::Vector3f position();

public:

private:
    enum ROBOTCMD {
        CMD_START_UDP, CMD_SERVO_ON, CMD_SERVO_OFF
    };
    bool sendCmd(QByteArray& cmd);
    bool executeCmd(const ROBOTCMD CmdID);
    inline QByteArray IntToArray(qint32 source);
    void MoveTread();
private slots:
    void init();
    void offline();
    void receivedMessage();
    void receivedError(QAbstractSocket::SocketError socketError);
    void readPendingDatagrams();

private:
#define MAX_ANGLE_INCREMENT 1.0
#define FRAME_LENGTH 80
    std::weak_ptr<MotomanRobotObserver> m_observer;
    std::shared_ptr<QTcpSocket> m_tcpSocket;
    std::shared_ptr<QUdpSocket> m_udpSocket;
    QString m_address;
    quint16 m_tcp_port;
    quint16 m_udp_port;
    bool m_executeSuccess;
    bool m_isOnline;
    Eigen::Vector3f m_translation;
    Eigen::Vector3f m_rotation;
    //joint的position, speed, accel.
    std::vector<double> m_currentPosition;
    std::vector<double> m_targetPosition;
    std::vector<double> m_speed;
    std::vector<double> m_accel;
    quint8 m_cmdID;
    std::shared_ptr<AbstractDigitIoDriver> m_digitInput;
    std::shared_ptr<AbstractDigitIoDriver> m_digitOutput;
    bool m_stop;
};

#endif //MOTOMAN_ROBOT_DRIVER_H
