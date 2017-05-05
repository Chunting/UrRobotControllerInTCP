//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "Ur10JointFilter.h"

Ur10JointFilter::Ur10JointFilter() {
}

Ur10JointFilter::~Ur10JointFilter() {
}

bool Ur10JointFilter::setup(const QString& configFilePath) {
    QJsonObject configJson;
    if (loadJson(configJson, configFilePath)) {
        m_jointLimits = readRealArray(configJson["JointLimits"]);
        for (auto& val : m_jointLimits) val = val * M_PI / 180 / 1000 * 8;
        COBOT_LOG.notice() << "Joint Filter Limits: " << putfixedfloats(6, 1, m_jointLimits, 180 / M_PI);
    }
    return true;
}

void Ur10JointFilter::applyFilter(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus) {
    m_target = target_;
    auto curPosi = ptrRobotStatus->q_actual;
    auto curVelo = ptrRobotStatus->qd_actual;

    if (m_target.size() != curPosi.size())
        return;

    double maxRadPerTick = 60 * M_PI / 180 / 1000 * 8;
    if (m_jointLimits.size() != m_target.size()) {
        m_jointLimits.resize(m_target.size(), maxRadPerTick);
        COBOT_LOG.warning() << "Config Joint Limit use Default: " << maxRadPerTick;
    }

    std::vector<double> diffNew(m_target.size(), 0);
    for (size_t i = 0; i < m_target.size(); i++) {
        diffNew[i] = m_target[i] - curPosi[i];

        if (diffNew[i] > m_jointLimits[i])
            diffNew[i] = m_jointLimits[i];
        if (diffNew[i] < -m_jointLimits[i])
            diffNew[i] = -m_jointLimits[i];
        target_[i] = curPosi[i] + diffNew[i];
    }

    //COBOT_LOG.debug() << putfixedfloats(7, 2, diffNew, 180 / M_PI);
}
