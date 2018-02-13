//
// Created by 潘绪洋 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SyncMover.h"

SyncMover::SyncMover() {
}

SyncMover::~SyncMover() {
}

void SyncMover::onMoveFinish(uint32_t moveId, MoveResult moveResult) {
    m_moveDataMutex.lock();
    auto iter = m_moveIdResult.find(moveId);
    if (iter != m_moveIdResult.end()) {
        iter->second = true;
        if (moveResult == MoveResult::Success) {
        } else {
            m_moveSuccess = false;
        }
        COBOT_LOG.debug("SyncMover") << std::setw(5) << iter->first << ", " << (int) moveResult;
    }

    bool allFinished = true;
    for (auto& iter : m_moveIdResult) {
        if (!iter.second) {
            allFinished = false;
            break;
        }
    }
    m_moveDataMutex.unlock();

    if (allFinished) {
        m_moveMsg.notify_all();
    }
}

bool SyncMover::move(const std::vector<RobotWaypoint>& movePoints) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (m_robotMover == nullptr)
        return false;

    m_moveSuccess = false;
    m_finishedMoveNum = 0;

    std::vector<uint32_t> moveIds;

    // Create move id
    m_moveDataMutex.lock();
    for (size_t i = 0; i < movePoints.size(); i++) {
        auto id = m_robotMover->generateMoveId();
        moveIds.push_back(id);
        m_moveIdResult[id] = false;
    }
    m_moveDataMutex.unlock();

    // Send move position data
    for (size_t i = 0; i < movePoints.size(); i++) {
        m_robotMover->move(moveIds[i], movePoints[i].position, movePoints[i].rpy);
    }

    // Wait result
    std::mutex waitMutex;
    std::unique_lock<std::mutex> waitLock(waitMutex);
    m_moveMsg.wait(waitLock);

    return m_moveSuccess;
}

void SyncMover::setArmRobotMover(const std::shared_ptr<AbstractArmRobotMoveDriver>& robotMover) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);
    m_robotMover = robotMover;
    m_robotMover->attach(std::dynamic_pointer_cast<ArmRobotMoveStatusObserver>(shared_from_this()));
}

void SyncMover::clearAttachedObject() {
    m_robotMover.reset();
}

bool SyncMover::setup(const QString& configFilePath) {
    return true;
}
