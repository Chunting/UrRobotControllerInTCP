//
// Created by zxj on 17-4-28.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_BINPICKINGDETECTORBOX_H
#define COBOTSYS_BINPICKINGDETECTORBOX_H

#include <cobotsys_abstract_binpicking_vision_detector.h>
#include "cloud_based_plane_segmentation.h"
#include "calibration.h"

using namespace cobotsys;
using namespace cobotsys::binpicking;

class  BinpickingDetectorBox : public AbstractBinpickingVisionDetector {
public:
    BinpickingDetectorBox();

    virtual ~BinpickingDetectorBox();

    virtual bool setup(const QString& configFilePath);

    virtual bool processVisionImage(const std::vector<VisionInputImage>& images);

    virtual bool getPickObjects(std::vector<BinObjGrabPose>& result) const;

private:
    calibration calbrat;

    VisionApi::PlaneSegment planeSegment;
};


#endif //COBOTSYS_BINPICKINGDETECTORBOX_H
