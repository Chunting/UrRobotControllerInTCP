//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "cobotsys_abstract_object_factory.h"


namespace cobotsys {
AbstractObjectFactory::AbstractObjectFactory() {
}

AbstractObjectFactory::~AbstractObjectFactory() {
    INFO_DESTRUCTOR(this);
}
}