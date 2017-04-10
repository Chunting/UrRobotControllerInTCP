//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <extra2.h>
#include "cobotsys_abstract_force_sensor.h"


namespace cobotsys {
	AbstractForceSensor::AbstractForceSensor() {
}

	AbstractForceSensor::~AbstractForceSensor() {
    INFO_DESTRUCTOR(this);
}

ForceSensorStreamObserver::ForceSensorStreamObserver() {
}

ForceSensorStreamObserver::~ForceSensorStreamObserver() {
    INFO_DESTRUCTOR(this);
}
}
