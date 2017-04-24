//
// Created by 潘绪洋 on 17-3-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_digit_io_driver.h"


namespace cobotsys {
AbstractDigitIoDriver::AbstractDigitIoDriver() {
}

AbstractDigitIoDriver::~AbstractDigitIoDriver() {
}

bool AbstractDigitIoDriver::setToolVoltage(double v) {
    return false;
}
}