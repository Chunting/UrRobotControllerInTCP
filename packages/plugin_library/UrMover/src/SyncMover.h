//
// Created by 潘绪洋 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef TUTORIALS_GENERALMOVEWAIT_H
#define TUTORIALS_GENERALMOVEWAIT_H

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <thread>
#include <deque>
#include <chrono>
#include <condition_variable>

using namespace cobotsys;


class SyncMover : public AbstractSyncMover, public ArmRobotMoveStatusObserver {
public:
    SyncMover();
    virtual ~SyncMover();

    virtual bool setup(const QString& configFilePath);
    virtual void onMoveFinish(uint32_t moveId, MoveResult moveResult);
    virtual bool move(const std::vector<RobotWaypoint>& movePoints);
    virtual void setArmRobotMover(const std::shared_ptr<AbstractArmRobotMoveDriver>& robotMover);

    virtual void clearAttachedObject();
protected:
    std::shared_ptr<AbstractArmRobotMoveDriver> m_robotMover;

    std::mutex m_mutex;

    uint32_t m_finishedMoveNum;
    std::condition_variable m_moveMsg;

    std::mutex m_moveDataMutex;
    std::map<uint32_t, bool> m_moveIdResult;
    bool m_moveSuccess;
};


#endif //TUTORIALS_GENERALMOVEWAIT_H
