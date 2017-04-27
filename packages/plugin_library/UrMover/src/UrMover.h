//
// Created by 潘绪洋 on 17-4-26.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_URMOVER_H
#define COBOTSYS_URMOVER_H

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <thread>
#include <deque>
#include <chrono>

using namespace cobotsys;

class UrMover : public AbstractArmRobotMoveDriver, public ArmRobotRealTimeStatusObserver {
public:
    UrMover();
    virtual ~UrMover();

    virtual bool setup(const QString& configFilePath);
    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy);
    virtual void attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer);
    virtual void setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver);
    virtual void setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver);

    virtual std::shared_ptr<AbstractKinematicSolver>& getKinematicSolver() { return m_kinematicSolver; }

    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

    virtual void clearAttachedObject();

    void moveProcess();

    virtual bool start();
    virtual void clearAll();

protected:
    std::vector<std::shared_ptr<ArmRobotMoveStatusObserver> > m_observers;
    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_realTimeDriver;
    std::shared_ptr<AbstractKinematicSolver> m_kinematicSolver;

//    Eigen::Matrix4d m_cam2base;
//    std::vector<double> m_initialJoint;
    std::vector<double> m_curJoint;
    std::mutex m_mutex;
    bool m_robotConnected;
    uint64_t m_curJointNum;

    struct MoveTarget {
        uint32_t moveId;
        cv::Point3d pos;
        cv::Vec3d rpy;
    };
    std::deque<MoveTarget> m_targets;
    bool m_clearMoveTarget;

    bool pickMoveTarget(MoveTarget& moveTarget);

    void notify(const MoveTarget& moveTarget);

    static std::vector<double> toVector(const MoveTarget& moveTarget);
    static double poseDiff(const std::vector<double>& a, const std::vector<double>& b);

    std::thread m_moverThread;
    bool m_exitLoop;
};


#endif //COBOTSYS_URMOVER_H
