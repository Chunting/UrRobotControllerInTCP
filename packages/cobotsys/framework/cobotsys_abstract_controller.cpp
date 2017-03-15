//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "cobotsys_abstract_controller.h"

cobotsys::AbstractController::AbstractController(){
}

cobotsys::AbstractController::~AbstractController(){
    INFO_DESTRUCTOR(this);
}

bool cobotsys::AbstractController::setup(const QString& configFilePath){
    return false;
}

namespace cobotsys {
AbstractControllerWidget::AbstractControllerWidget()
        : QWidget(nullptr){
}

AbstractControllerWidget::~AbstractControllerWidget(){
    INFO_DESTRUCTOR(this);
}

bool AbstractControllerWidget::setup(const QString& configFilePath){
    return false;
}
}
