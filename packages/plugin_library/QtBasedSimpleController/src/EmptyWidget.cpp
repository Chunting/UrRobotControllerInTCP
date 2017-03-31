//
// Created by 潘绪洋 on 17-3-15.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "EmptyWidget.h"

EmptyWidget::EmptyWidget(){
}

EmptyWidget::~EmptyWidget(){
}


bool EmptyWidget::start(){
    return false;
}

void EmptyWidget::pause(){
}

void EmptyWidget::stop(){
}

bool EmptyWidget::setup(const QString& configFilePath){
    return true;
}
