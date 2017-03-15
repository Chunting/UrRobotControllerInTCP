//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_controller.h"

cobotsys::AbstractController::AbstractController(){
}

cobotsys::AbstractController::~AbstractController(){
}

bool cobotsys::AbstractController::setup(const QString& configFilePath){
    return false;
}

namespace cobotsys {
AbstractControllerWidget::AbstractControllerWidget()
        : QWidget(nullptr){
}

AbstractControllerWidget::~AbstractControllerWidget(){
}

bool AbstractControllerWidget::setup(const QString& configFilePath){
    return false;
}
}
