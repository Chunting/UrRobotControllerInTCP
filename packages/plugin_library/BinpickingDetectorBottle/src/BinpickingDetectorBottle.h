//
// Created by zxj on 17-4-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_BINPICKINGDETECTORBOTTLE_H
#define COBOTSYS_BINPICKINGDETECTORBOTTLE_H

#include <cobotsys_abstract_binpicking_vision_detector.h>
#include "calibration.h"
#include "detector.h"

using namespace cobotsys;
using namespace cobotsys::binpicking;

class  BinpickingDetectorBottle : public AbstractBinpickingVisionDetector {
public:
    BinpickingDetectorBottle();

    virtual ~BinpickingDetectorBottle();

    virtual bool setup(const QString& configFilePath);

    virtual bool processVisionImage(const std::vector<VisionInputImage>& images);

    virtual bool getPickObjects(std::vector<BinObjGrabPose>& result) const;

private:
    calibration calbrat;

    Detector detector;

    std::vector<Eigen::Vector3d> positions;
};


#endif //COBOTSYS_BINPICKINGDETECTORBOTTLE_H
