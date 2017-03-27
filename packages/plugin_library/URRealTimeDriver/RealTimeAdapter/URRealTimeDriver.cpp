//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "URRealTimeDriver.h"


URRealTimeDriver::URRealTimeDriver(){
}

URRealTimeDriver::~URRealTimeDriver(){
}

void URRealTimeDriver::move(const std::vector<double>& q){
}

std::shared_ptr<AbstractDigitIoDriver> URRealTimeDriver::getDigitIoDriver(int deviceId){
    return nullptr;
}

void URRealTimeDriver::attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer){
}

bool URRealTimeDriver::start(){
    return false;
}

void URRealTimeDriver::stop(){
}

bool URRealTimeDriver::setup(const QString& configFilePath){
    return false;
}
