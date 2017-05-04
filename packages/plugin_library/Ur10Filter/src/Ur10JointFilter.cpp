//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "Ur10JointFilter.h"

Ur10JointFilter::Ur10JointFilter() {
}

Ur10JointFilter::~Ur10JointFilter() {
}

bool Ur10JointFilter::setup(const QString& configFilePath) {
    return true;
}

void Ur10JointFilter::applyFilter(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus) {
    m_target = target_;
    auto curPosi = ptrRobotStatus->q_actual;

    double maxRadPerTick = 60 * M_PI / 180 / 1000 * 8;

    std::vector<double> diffNew(m_target.size(), 0);
    for (size_t i = 0; i < m_target.size(); i++) {
        diffNew[i] = m_target[i] - curPosi[i];

        if (diffNew[i] > maxRadPerTick)
            diffNew[i] = maxRadPerTick;
        if (diffNew[i] < -maxRadPerTick)
            diffNew[i] = -maxRadPerTick;
        target_[i] = curPosi[i] + diffNew[i];
    }

    COBOT_LOG.debug() << putfixedfloats(7, 2, diffNew, 180 / M_PI);
}
