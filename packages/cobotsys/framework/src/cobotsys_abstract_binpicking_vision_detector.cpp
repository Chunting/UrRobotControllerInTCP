//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_binpicking_vision_detector.h"


namespace cobotsys {
namespace binpicking {
AbstractBinpickingVisionDetector::AbstractBinpickingVisionDetector() {
}

AbstractBinpickingVisionDetector::~AbstractBinpickingVisionDetector() {
}

void AbstractBinpickingVisionDetector::debugMat(const std::string& winName, const cv::Mat& mat) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    auto& iter = m_debugImages[winName];
    iter.isUpdated = true;
    iter.winName = winName;
    iter.matData = mat;
}

void AbstractBinpickingVisionDetector::debugRenderer() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    for (auto& iter : m_debugImages) {
        if (iter.second.isUpdated) {
            iter.second.isUpdated = false;
            cv::imshow(iter.second.winName,
                       iter.second.matData);
        }
    }
}
}
}