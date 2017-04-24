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
    Reset = 0,
    Set = 1,
};

enum class DigitIoPort {
    Port_0,
    Port_1,
    Port_2,
    Port_3,
    Port_4,
    Port_5,
    Port_6,
    Port_7,

    Port_8,
    Port_9,
    Port_10,
    Port_11,
    Port_12,
    Port_13,
    Port_14,
    Port_15,

    Port_16,
    Port_Ur_Tool_In_0 = (int) Port_16,
    Port_17
};

Q_DECLARE_FLAGS(DigitIoPorts, DigitIoPort);

class AbstractDigitIoDriver : public AbstractObject {
public:
    AbstractDigitIoDriver();
    virtual ~AbstractDigitIoDriver();

    /**
     * 用于设置IO口的状态为ON
     * @param ioPorts 可以是 DigitIoPort的组合
     */
    virtual void setIo(DigitIoPorts ioPorts, DigitIoStatus ioStatus) = 0;
    /**
     * 获取指定IO口状态。
     * @param ioPort
     * @return
     */
    virtual DigitIoStatus getIoStatus(DigitIoPort ioPort) = 0;

    virtual bool isDigitInput() const = 0;
    virtual bool isDigitOutput() const = 0;

    virtual bool setToolVoltage(double v);
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_DIGIT_IO_DRIVER_H
