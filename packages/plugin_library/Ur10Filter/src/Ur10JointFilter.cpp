//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "Ur10JointFilter.h"
#include <cxx/cxx.h>

Ur10JointFilter::Ur10JointFilter() {
}

Ur10JointFilter::~Ur10JointFilter() {
}

bool Ur10JointFilter::setup(const QString& configFilePath) {
    QJsonObject configJson;
    if (loadJson(configJson, configFilePath)) {
        m_jointLimits = readRealArray(configJson["JointLimits"]);
        m_jointAccLimits = readRealArray(configJson["JointAccLimits"]);
        cxx::scale_all(m_jointLimits, M_PI / 180 / 1000 * 8);
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

    if (cxx::distance(curVelo) > 0.001) {
//        COBOT_LOG.debug() << "J-Speed: " << putfixedfloats(6, 3, curVelo, 180 / M_PI);
    }

    double maxRadPerTick = 360 * M_PI / 180 / 1000 * 8;
    if (m_jointLimits.size() != m_target.size()) {
        m_jointLimits.resize(m_target.size(), maxRadPerTick);
        COBOT_LOG.warning() << "Config Joint Limit use Default: " << maxRadPerTick;
    }

    auto posiInc = curVelo;
    std::vector<double> diffNew(m_target.size(), 0);

    cxx::norm_all(posiInc);
    cxx::add_other(posiInc, m_jointAccLimits);

    for (size_t i = 0; i < m_target.size(); i++) {
        diffNew[i] = m_target[i] - curPosi[i];

        cxx::limit(diffNew[i], posiInc[i]);

        cxx::limit(diffNew[i], m_jointLimits[i]);

        target_[i] = curPosi[i] + diffNew[i];
    }
}
