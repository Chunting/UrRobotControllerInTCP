//
// Created by 潘绪洋 on 17-3-13.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_DIGIT_IO_DRIVER_H
#define PROJECT_COBOTSYS_ABSTRACT_DIGIT_IO_DRIVER_H


#include <QFlags>
#include "cobotsys_abstract_object.h"

namespace cobotsys {

enum class DigitIoStatus {
    Reset,
    Set,
};

enum class DigitIoPort {
    Port_1 = 1UL << 1,
    Port_2 = 1UL << 2,
    Port_3 = 1UL << 3,
    Port_4 = 1UL << 4,
    Port_5 = 1UL << 5,
    Port_6 = 1UL << 6,
    Port_7 = 1UL << 7,
    Port_8 = 1UL << 8,
};

Q_DECLARE_FLAGS(DigitIoPorts, DigitIoPort);

class AbstractDigitIoDriver : public AbstractObject {
public:
    AbstractDigitIoDriver();
    virtual ~AbstractDigitIoDriver();

    virtual void setIo(DigitIoPorts ioPorts) = 0;
    virtual void resetIo(DigitIoPorts ioPorts) = 0;
    virtual DigitIoStatus getIoStatus(DigitIoPort ioPort) = 0;
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_DIGIT_IO_DRIVER_H
