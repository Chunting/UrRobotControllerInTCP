//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_VISION_ALGORITHM_H
#define COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_VISION_ALGORITHM_H

#include "cobotsys_abstract_object.h"

namespace cobotsys {
namespace binpicking {

class AbstractBinpickingVisionDetector : public AbstractObject {
public:
    AbstractBinpickingVisionDetector();
    virtual ~AbstractBinpickingVisionDetector();

    virtual void processVisionImage() = 0;

};
}
}


#endif //COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_VISION_ALGORITHM_H
