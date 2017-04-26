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
    Port_0 = 1UL << 0,
    Port_1 = 1UL << 1,
    Port_2 = 1UL << 2,
    Port_3 = 1UL << 3,
    Port_4 = 1UL << 4,
    Port_5 = 1UL << 5,
    Port_6 = 1UL << 6,
    Port_7 = 1UL << 7,

    Port_8 = 1UL << 8,
    Port_9 = 1UL << 9,
    Port_10 = 1UL << 10,
    Port_11 = 1UL << 11,
    Port_12 = 1UL << 12,
    Port_13 = 1UL << 13,
    Port_14 = 1UL << 14,
    Port_15 = 1UL << 15,

    Port_16 = 1UL << 16,
    Port_Ur_Tool_In_0 = (int) Port_16,
    Port_17 = 1UL << 17
};

Q_DECLARE_FLAGS(DigitIoPorts, DigitIoPort);

class AbstractDigitIoDriver : public AbstractObject {
public:
    AbstractDigitIoDriver();
    virtual ~AbstractDigitIoDriver();

    /**
     * 端口状态是否正常的打开了。
     */
    virtual bool isOpened() const = 0;
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
