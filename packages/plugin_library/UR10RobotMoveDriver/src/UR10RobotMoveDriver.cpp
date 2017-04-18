//
// Created by eleven on 17-4-14.
//

#include "UR10RobotMoveDriver.h"
#include "cobotsys_abstract_kinematic_solver.h"
#include "cobotsys_logger.h"

class UR10RobotRealTimeStatusObserver : public ArmRobotRealTimeStatusObserver {
public:
    UR10RobotRealTimeStatusObserver(const std::shared_ptr<UR10RobotMoveDriver>& robotMoveDriver)
            : m_robotMoveDriver(robotMoveDriver){ }


    virtual void onArmRobotConnect() { }

    virtual void onArmRobotDisconnect() { }

    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus)
    {
        std::lock_guard<std::mutex> lock(m_robotMoveDriver.lock()->m_mutex);
        m_robotMoveDriver.lock()->m_initialJoint = ptrRobotStatus->q_actual;
    }

private:
    std::weak_ptr<UR10RobotMoveDriver> m_robotMoveDriver;
};

UR10RobotMoveDriver::UR10RobotMoveDriver()
{
    m_realTimeDriver = nullptr;
}

UR10RobotMoveDriver::~UR10RobotMoveDriver()
{
    if (m_realTimeDriver)
        m_realTimeDriver->stop();
}

bool UR10RobotMoveDriver::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy)
{
    if (!m_realTimeDriver)
    {
        COBOT_LOG.error() << "The RealTimeDriver is NULL";
        return false;
    }

    Eigen::Vector4d camPos;
    camPos << pos.x, pos.y, pos.z, 1;
    Eigen::Vector4d basePos = m_cam2base * camPos;


    std::vector<double> targetPos;
    targetPos.push_back(basePos(0));
    targetPos.push_back(basePos(1));
    targetPos.push_back(basePos(2));
    targetPos.push_back(rpy[0]);
    targetPos.push_back(rpy[1]);
    targetPos.push_back(rpy[2]);

    std::vector<double> targetJoint;
    std::vector<double> initialJoint;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        initialJoint = m_initialJoint;
    }
    m_kinematicSolver->cartToJnt(initialJoint, targetPos, targetJoint);
    m_realTimeDriver->move(targetJoint);
    m_realTimeDriver->start();

    notify(moveId);
}

//在配置文件中读取手眼标定结果，并转换为旋转矩阵
bool UR10RobotMoveDriver::setup(const QString& configFilePath)
{
    cv::FileStorage fs(configFilePath.toLocal8Bit().constData(), cv::FileStorage::READ);

    if (fs.isOpened()) {

        Eigen::Quaterniond rot_cam2base;
        Eigen::Vector3d vec_cam2base;

        rot_cam2base.x() = fs["RotationX"];
        rot_cam2base.y() = fs["RotationY"];
        rot_cam2base.z() = fs["RotationZ"];
        rot_cam2base.w() = fs["RotationW"];

        vec_cam2base.x() = fs["TranslationX"];
        vec_cam2base.y() = fs["TranslationY"];
        vec_cam2base.z() = fs["TranslationZ"];

        Eigen::Matrix3d rot = rot_cam2base.toRotationMatrix();
        m_cam2base.block(0, 0, 3, 3) = rot;
        m_cam2base.row(3) << 0, 0, 0, 1;
        m_cam2base.col(3) << vec_cam2base(0), vec_cam2base(1), vec_cam2base(2), 1;

        return true;
    } else {
        return false;
    }
}

void UR10RobotMoveDriver::attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer)
{
    if (observer && find(m_observers.begin(), m_observers.end(), observer) == m_observers.end())
        m_observers.push_back(observer);
}

void UR10RobotMoveDriver::setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver)
{
    m_realTimeDriver = realTimeDriver;

    if (!m_realTimeDriver)
        return;


    std::shared_ptr<UR10RobotRealTimeStatusObserver> robotStatusObserver
            = std::make_shared<UR10RobotRealTimeStatusObserver>(std::dynamic_pointer_cast<UR10RobotMoveDriver>(shared_from_this()));
    m_realTimeDriver->attach(robotStatusObserver);
}

void UR10RobotMoveDriver::notify(uint32_t moveId)
{
    for (auto& observer : m_observers) {
        observer->onMoveFinish(moveId);
    }
}
