//
// Created by 潘绪洋 on 17-3-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_gripper.h"
#include "extra2.h"

namespace cobotsys {
GripperStatusObserver::GripperStatusObserver(){
}

GripperStatusObserver::~GripperStatusObserver(){
    INFO_DESTRUCTOR(this);
}

AbstractGripper::AbstractGripper(){
}

AbstractGripper::~AbstractGripper(){
    INFO_DESTRUCTOR(this);
}
}