//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include <cobotsys_qt.h>
#include "cobotsys_abstract_object.h"


namespace cobotsys {
AbstractObject::AbstractObject(){
}

AbstractObject::~AbstractObject(){
}

bool AbstractObject::setup(const QString& configFilePath){
    COBOT_LOG.warning() << "Empty Config: " << configFilePath;
    return true;
}
}