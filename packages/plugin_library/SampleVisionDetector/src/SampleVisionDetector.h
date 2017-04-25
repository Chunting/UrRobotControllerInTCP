//
// Created by 潘绪洋 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_SAMPLEVISIONDETECTOR_H
#define COBOTSYS_SAMPLEVISIONDETECTOR_H

#include <cobotsys_abstract_binpicking_vision_detector.h>

using namespace cobotsys;
using namespace cobotsys::binpicking;

class SampleVisionDetector : public AbstractBinpickingVisionDetector {
public:
    SampleVisionDetector();
    virtual ~SampleVisionDetector();
    virtual bool setup(const QString& configFilePath);
    virtual bool processVisionImage(const std::vector<VisionInputImage>& images);
    virtual bool getPickObjects(std::vector<BinObjGrabPose>& result) const;
};


#endif //COBOTSYS_SAMPLEVISIONDETECTOR_H
