//
// Created by zhangshaohua on 17-4-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_ENCODER_H_H
#define COBOTSYS_COBOTSYS_ABSTRACT_ENCODER_H_H
#include "cobotsys_abstract_object.h"

namespace cobotsys {
class AbstractEncoder : public AbstractObject {
public:
    AbstractEncoder();
    virtual ~AbstractEncoder();
};
}
#endif //COBOTSYS_COBOTSYS_ABSTRACT_ENCODER_H_H
