//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "EpsonRobotDriver.h"

EpsonRobotDriver::EpsonRobotDriver(){
}

EpsonRobotDriver::~EpsonRobotDriver(){
}

void EpsonRobotDriver::move(const std::vector<double>& q){

}

std::shared_ptr<AbstractDigitIoDriver> EpsonRobotDriver::getDigitIoDriver(int deviceId){
    return nullptr;
}

void EpsonRobotDriver::attach(std::shared_ptr<ArmRobotRealTimeStatusObserver> observer){
}

bool EpsonRobotDriver::start(){
    return false;
}

void EpsonRobotDriver::stop(){
}

bool EpsonRobotDriver::setup(const QString& configFilePath){
    return false;
}
