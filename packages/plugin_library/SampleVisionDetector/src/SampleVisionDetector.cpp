//
// Created by 潘绪洋 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "SampleVisionDetector.h"
#include <chrono>
#include <thread>

SampleVisionDetector::SampleVisionDetector() {
}

SampleVisionDetector::~SampleVisionDetector() {
}

bool SampleVisionDetector::setup(const QString& configFilePath) {
    return true;
}

bool SampleVisionDetector::processVisionImage(const std::vector<VisionInputImage>& images) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

bool SampleVisionDetector::getPickObjects(std::vector<BinObjGrabPose>& result) const {
    return true;
}
