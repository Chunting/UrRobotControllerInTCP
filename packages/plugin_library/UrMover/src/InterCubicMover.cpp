//
// Created by 潘绪洋 on 17-5-12.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "InterCubicMover.h"
#include <extra2.h>
#include <cxx/cxx.h>

InterCubicMover::InterCubicMover() {
    m_robotConnected = true;
    m_exitLoop = false;
    m_timeMoveScale_ = 3;
}

InterCubicMover::~InterCubicMover() {
    m_exitLoop = true;
    if (m_moverThread.joinable()) {
        m_moverThread.join();
    }
}

bool InterCubicMover::setup(const QString& configFilePath) {
    return true;
}

bool InterCubicMover::move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_robotConnected) {
        m_targets.push_back({moveId, pos, rpy});
        COBOT_LOG.debug("InCb") << "Total Targets : " << m_targets.size();
        return true;
    } else {
    }
    return false;
}

bool InterCubicMover::move(const std::vector<RobotWaypoint>& waypoints) {
    // TODO 计算路径，规划完整的Joint
    return false;
}

void InterCubicMover::attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    for (auto& iter : m_observers) {
        if (iter == observer) {
            return;
        }
    }
    m_observers.push_back(observer);
}

void InterCubicMover::setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_realTimeDriver = realTimeDriver;
    m_realTimeDriver->attach(std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this()));
}

void InterCubicMover::setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_kinematicSolver = kinematicSolver;
}


void InterCubicMover::onArmRobotConnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = true;
}

void InterCubicMover::onArmRobotDisconnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = false;
    clearAll();
}

void InterCubicMover::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_curJoint = ptrRobotStatus->q_actual;
    m_curJointVel = ptrRobotStatus->qd_actual;
}

void InterCubicMover::notify(const InterCubicMover::MoveTarget& moveTarget, MoveResult moveResult) {
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


std::vector<double> interp_cubic(std::vector<double>& p0_pos, std::vector<double>& p1_pos,
                                 std::vector<double>& p0_vel, std::vector<double>& p1_vel,
                                 double t = 0.008, double T = 1) {
    /*Returns positions of the joints at time 't' */
    std::vector<double> positions;
    for (unsigned int i = 0; i < p0_pos.size(); i++) {
        double a = p0_pos[i];
        double b = p0_vel[i];
        double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i] - T * p1_vel[i]) / pow(T, 2);
        double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i] + T * p1_vel[i]) / pow(T, 3);
        positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
    }
    return positions;
}

void filter_move_joint_target(std::vector<double>& target_) {
    for (auto& iter : target_) {
        if (iter > M_PI * 1.9) iter -= M_PI * 2;
        if (iter < -M_PI * 1.9) iter += M_PI * 2;
    }
}

void InterCubicMover::moveProcess() {
    auto timePoint = std::chrono::high_resolution_clock::now();

    std::vector<double> curPose;
    std::vector<double> targetJoint;
    MoveTarget moveTarget;
    std::vector<double> jnt_posi, src_posi;


    COBOT_LOG.notice() << "InterCubicMover is running.";
    auto hres_start = std::chrono::high_resolution_clock::now();
    while (!m_exitLoop) {
        timePoint = timePoint + std::chrono::milliseconds(1);

        if (pickMoveTarget(moveTarget)) {
            auto movePose = toVector(moveTarget);

            if (m_kinematicSolver->cartToJnt(m_curJoint, movePose, targetJoint) == 0) {
                filter_move_joint_target(targetJoint);
                m_kinematicSolver->jntToCart(m_curJoint, src_posi);
                auto jointSpaceDiff = targetJoint;
                for (size_t i = 0; i < targetJoint.size(); i++) {
                    jointSpaceDiff[i] = src_posi[i] - movePose[i];
                    if (i > 2) jointSpaceDiff[i] = 0;
                }
                auto jointSpaceLen = cxx::distance(jointSpaceDiff);


                auto moveTimeBegin = std::chrono::high_resolution_clock::now();
                std::vector<double> targetJointVel(targetJoint.size(), 0);

                m_mutex.lock();
                auto move_begin_joint = m_curJoint;
                auto move_begin_joint_vel = m_curJointVel;
                m_mutex.unlock();

                double move_time_max_ = jointSpaceLen / 1.0 * m_timeMoveScale_;
                if (move_time_max_ > 3.5) move_time_max_ = 3.5;

                COBOT_LOG.notice("JointSpaceLen") << jointSpaceLen << ", " << move_time_max_;

                // Move Until Arrival Target
                while (isTargetNotArrival(moveTarget)) {
                    std::chrono::duration<double> delta_t_
                            = std::chrono::high_resolution_clock::now() - moveTimeBegin;

                    std::vector<double> positions_;
                    if (delta_t_.count() < move_time_max_) {
                        positions_ = interp_cubic(move_begin_joint, targetJoint,
                                                  move_begin_joint_vel, targetJointVel, delta_t_.count(),
                                                  move_time_max_);
                    } else {
                        positions_ = targetJoint;
//                        COBOT_LOG.notice("positions_") << "Cubic Joint Over.";
                    }
//                    calcStepJointDegree(positions_);
                    m_mutex.lock();
                    auto posiDiff = m_curJoint;
                    m_mutex.unlock();
                    for (size_t i = 0; i < posiDiff.size(); i++) posiDiff[i] = positions_[i] - posiDiff[i];
//                    COBOT_LOG.debug("positions_") << putfixedfloats(6, 1, posiDiff, 180 / M_PI);
                    m_realTimeDriver->move(positions_);

                    std::this_thread::sleep_until(timePoint);

                    timePoint += std::chrono::milliseconds(2);
                    if (m_exitLoop) {
                        break;
                    }
                }
            } else {
                COBOT_LOG.error() << "Target: " << moveTarget.pos << ", " << moveTarget.rpy
                                  << " Can Not Reached!";
                notify(moveTarget, MoveResult::InvalidMoveTarget);
                clearAll();
            }
        }
        std::this_thread::sleep_until(timePoint);
    }
    COBOT_LOG.notice() << "InterCubicMover is stopped.";
}

void InterCubicMover::clearAttachedObject() {
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

bool InterCubicMover::pickMoveTarget(InterCubicMover::MoveTarget& moveTarget) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_targets.size() && m_robotConnected) {
        moveTarget = m_targets.front();
        moveTarget.begin_time = std::chrono::high_resolution_clock::now();
        m_targets.pop_front();
        m_poseLastError = std::numeric_limits<double>::max();
        return true;
    }
    return false;
}

std::vector<double> InterCubicMover::toVector(const InterCubicMover::MoveTarget& moveTarget) {
    std::vector<double> j;
    j.push_back(moveTarget.pos.x);
    j.push_back(moveTarget.pos.y);
    j.push_back(moveTarget.pos.z);
    j.push_back(moveTarget.rpy[0]);
    j.push_back(moveTarget.rpy[1]);
    j.push_back(moveTarget.rpy[2]);
    return j;
}

double InterCubicMover::poseDiff(const std::vector<double>& a, const std::vector<double>& b) {
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

bool InterCubicMover::start() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_kinematicSolver && m_realTimeDriver) {
        m_moverThread = std::thread(&InterCubicMover::moveProcess, this);
        return true;
    }
    return false;
}

void InterCubicMover::clearAll() {
    std::deque<MoveTarget> tmpTargets;

    COBOT_LOG.notice() << "InterCubicMover: Clear ALL";

    m_mutex.lock();
    tmpTargets = m_targets;
    m_targets.clear();
    m_mutex.unlock();

    for (auto& iter : tmpTargets) {
        notify(iter, MoveResult::Cancled);
    }
}

bool InterCubicMover::isTargetNotArrival(const InterCubicMover::MoveTarget& moveTarget) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    std::vector<double> curPose;

    m_kinematicSolver->jntToCart(m_curJoint, curPose);

    auto pose = toVector(moveTarget);
    auto actual_pose_diff = poseDiff(pose, curPose); // 目标与实际的位置误差
    auto error_prev = std::fabs(actual_pose_diff - m_poseLastError);

    m_poseLastError = actual_pose_diff;

    bool isPoseErrorGood = actual_pose_diff < 0.005;
    bool isPoseErrorNoChanged = (actual_pose_diff < 1 && error_prev < 0.001);

    if (isPoseErrorGood || isPoseErrorNoChanged) {
        // Target Arrival

        std::chrono::duration<double> time_diff = std::chrono::high_resolution_clock::now() - moveTarget.begin_time;

        notify(moveTarget, MoveResult::Success);
        COBOT_LOG.notice("CubicMV")
                << std::setw(5) << moveTarget.moveId
                << " Mover Target: " << moveTarget.pos << ", " << moveTarget.rpy
                << " Finished. Time: "
                << time_diff.count() * 1000 << "ms";
        return false;
    }

    if (m_exitLoop || !m_robotConnected) {
        notify(moveTarget, MoveResult::Cancled);
    }
    return true;
}

double InterCubicMover::calcStepJointDegree(const std::vector<double>& targetJoint_) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);

    auto diff_ = targetJoint_;
    for (size_t i = 0; i < diff_.size(); i++) {
        diff_[i] = targetJoint_[i] - m_curJoint[i];
    }
    COBOT_LOG.debug("JntDiff") << putfixedfloats(6, 1, diff_, 180 / M_PI);
    return 0;
}
