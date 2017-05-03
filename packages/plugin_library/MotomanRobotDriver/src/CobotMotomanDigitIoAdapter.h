//
// Created by 杨帆 on 17-5-2.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOT_MOTOMAN_DIGIT_IO_ADAPTER_H
#define COBOT_MOTOMAN_DIGIT_IO_ADAPTER_H


#include <cobotsys_abstract_digit_io_driver.h>
#include "CobotMotomanRealTimeCommCtrl.h"

using namespace cobotsys;

class CobotMotomanDigitIoAdapter : public AbstractDigitIoDriver {
public:
    CobotMotomanDigitIoAdapter();
    virtual ~CobotMotomanDigitIoAdapter();

    virtual bool setup(const QString& configFilePath);

    virtual bool isOpened() const;
    virtual void setIo(DigitIoPorts ioPorts, DigitIoStatus ioStatus);
    virtual DigitIoStatus getIoStatus(DigitIoPort ioPort);
    virtual bool isDigitInput() const;
    virtual bool isDigitOutput() const;

    virtual bool setToolVoltage(double v);


    void setMotomanRealTimeCtrl(CobotMotomanRealTimeCommCtrl* realTimeCommCtrl);


    // 这是一个内部类，没有怎么严格的要求写。
    std::vector<DigitIoStatus> m_ioPortStatus;
    bool m_isInput;
    bool m_isOutput;

    int m_inputIoStatus;
    int m_outputIoStatus;

    int m_debugIoLastStatus;

    CobotMotomanRealTimeCommCtrl* m_realTimeCommCtrl;

    void debugIoStatus();

    void setDigitOut(int portIndex, bool b);

};


#endif //COBOT_MOTOMAN_DIGIT_IO_ADAPTER_H
