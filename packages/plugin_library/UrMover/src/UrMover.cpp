//
// Created by 潘绪洋 on 17-4-26.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "UrMover.h"
#include <extra2.h>

UrMover::UrMover() {
    m_robotConnected = true;
    m_curJointNum = 0;
    m_exitLoop = false;
}

UrMover::~UrMover() {
    m_exitLoop = true;
    if (m_moverThread.joinable()) {
        m_moverThread.join();
    }
}

bool UrMover::setup(const QString& configFilePath) {
    return true;
}

bool UrMover::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_robotConnected) {
        m_targets.push_back({moveId, pos, rpy});
        return true;
    }
    return false;
}

void UrMover::attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    for (auto& iter : m_observers) {
        if (iter == observer) {
            return;
        }
    }
    m_observers.push_back(observer);
}

void UrMover::setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver) {
    m_realTimeDriver = realTimeDriver;
    m_realTimeDriver->attach(std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this()));
}

void UrMover::setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver) {
    m_kinematicSolver = kinematicSolver;
}


void UrMover::onArmRobotConnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = true;
}

void UrMover::onArmRobotDisconnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = false;
}

void UrMover::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_curJoint = ptrRobotStatus->q_actual;
    m_curJointNum++;
}

void UrMover::notify(const UrMover::MoveTarget& moveTarget, MoveResult moveResult) {
    std::vector<std::shared_ptr<ArmRobotMoveStatusObserver> > observers;
    m_mutex.lock();
    observers = m_observers;
    m_mutex.unlock();

    for (auto& ob : observers) {
        if (ob) {
            ob->onMoveFinish(moveTarget.moveId, moveResult);
        }
    }
}

void UrMover::moveProcess() {
    auto timePoint = std::chrono::high_resolution_clock::now();
    std::vector<double> joint;
    std::vector<double> curPose;
    std::vector<double> targetJoint;
    uint64_t jointNum = 0;
    uint64_t jointNumOld = 0;
    MoveTarget moveTarget;
    bool moveFinished = true;
    bool noMoveTarget = true;

    double actual_pose_diff;
    double pose_err_last = 0;

    COBOT_LOG.notice() << "UrMover is running.";
    auto hres_start = std::chrono::high_resolution_clock::now();
    while (!m_exitLoop) {
        timePoint = timePoint + std::chrono::milliseconds(1);
        m_mutex.lock();
        jointNum = m_curJointNum;
        joint = m_curJoint;
        m_mutex.unlock();

        m_kinematicSolver->jntToCart(joint, curPose);

        if (m_clearMoveTarget) {
            m_clearMoveTarget = false;
            if (!noMoveTarget) {
                noMoveTarget = true;
                moveFinished = true;
                COBOT_LOG.notice() << "Mover Target has canceled, " << jointNum;
                hres_start = std::chrono::high_resolution_clock::now();
                notify(moveTarget, MoveResult::Cancled);
            }
        }

        if (jointNum > jointNumOld) {
            // Pick target
            if (noMoveTarget) {
                if (pickMoveTarget(moveTarget)) {
                    hres_start = std::chrono::high_resolution_clock::now();
                    moveFinished = false;
                    noMoveTarget = false;
                    COBOT_LOG.notice() << std::setw(5) << moveTarget.moveId
                                       << " Mover Target: " << moveTarget.pos << ", " << moveTarget.rpy;
                }
            }

            // Go Target
            if (!noMoveTarget) {
                auto pose = toVector(moveTarget);
                actual_pose_diff = poseDiff(pose, curPose); // 目标与实际的位置误差

                auto error_prev = fabs(actual_pose_diff - pose_err_last);

//                COBOT_LOG.debug() << "Mover Error: " << actual_pose_diff << ", " << error_prev;
                if (actual_pose_diff < 0.005 ||
                    (actual_pose_diff < 1 && error_prev < 0.001)) {
                    if (!moveFinished) {
                        auto hres_cur = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> time_diff = hres_cur - hres_start;
                        moveFinished = true;
                        noMoveTarget = true;

                        notify(moveTarget, MoveResult::Success);
                        COBOT_LOG.notice() << std::setw(5) << moveTarget.moveId
                                           << " Mover Target: " << moveTarget.pos << ", " << moveTarget.rpy
                                           << " Finished. Time: " << time_diff.count() * 1000 << "ms";
                    }
                } else {
                    // Calc move joint
                    if (m_kinematicSolver->cartToJnt(joint, pose, targetJoint) == 0) {

                        // TODO smooth target joint commands
                        m_realTimeDriver->move(targetJoint);
                    } else {
                        COBOT_LOG.error() << "Target: " << moveTarget.pos << ", " << moveTarget.rpy
                                          << " Can Not Reached!";
                        notify(moveTarget, MoveResult::InvalidMoveTarget);
                        clearAll();
                    }
                }
                pose_err_last = actual_pose_diff;
            }
//            auto debugger = COBOT_LOG.debug();
//            auto jnt = joint;
//            for (int i = 0; i < jnt.size(); i++) jnt[i] *= 180 / M_PI;
////            debugger << "tcp: " << curPose << ", Jnt " << jnt;
        }
        jointNumOld = jointNum;
        std::this_thread::sleep_until(timePoint);
    }
    COBOT_LOG.notice() << "UrMover is stopped.";
}

void UrMover::clearAttachedObject() {
    m_mutex.lock();
    m_exitLoop = true;
    m_mutex.unlock();

    if (m_moverThread.joinable()) {
        m_moverThread.join();
    }

    m_mutex.lock();
    m_observers.clear();
    m_kinematicSolver.reset();
    m_realTimeDriver.reset();
    m_mutex.unlock();
}

bool UrMover::pickMoveTarget(UrMover::MoveTarget& moveTarget) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_targets.size()) {
        moveTarget = m_targets.front();
        m_targets.pop_front();
        return true;
    }
    return false;
}

std::vector<double> UrMover::toVector(const UrMover::MoveTarget& moveTarget) {
    std::vector<double> j;
    j.push_back(moveTarget.pos.x);
    j.push_back(moveTarget.pos.y);
    j.push_back(moveTarget.pos.z);
    j.push_back(moveTarget.rpy[0]);
    j.push_back(moveTarget.rpy[1]);
    j.push_back(moveTarget.rpy[2]);
    return j;
}

double UrMover::poseDiff(const std::vector<double>& a, const std::vector<double>& b) {
    double diff_sum = 0;
    if (a.size() == b.size()) {
        std::vector<double> diff(a.size(), 0);
        for (size_t i = 0; i < a.size(); i++) {
            diff[i] = a[i] - b[i];
            if (i >= 3) {
                diff[i] = diff[i] * 180 / M_PI;
            } else {
                diff[i] = diff[i] * 1000;
            }
            diff[i] = diff[i] * diff[i];

            diff_sum += diff[i];
        }
    }
    return sqrt(diff_sum);
}

bool UrMover::start() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_kinematicSolver && m_realTimeDriver) {
        m_moverThread = std::thread(&UrMover::moveProcess, this);
        return true;
    }
    return false;
}

void UrMover::clearAll() {
    std::deque<MoveTarget> tmpTargets;

    m_mutex.lock();
    tmpTargets = m_targets;
    m_targets.clear();
    m_clearMoveTarget = true;
    m_mutex.unlock();

    for (auto& iter : tmpTargets) {
        notify(iter, MoveResult::Cancled);
    }
}
