//
// Created by 潘绪洋 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTURDIGITIOADAPTER_H
#define COBOTSYS_COBOTURDIGITIOADAPTER_H


#include <cobotsys_abstract_digit_io_driver.h>
#include "CobotUrRealTimeCommCtrl.h"

using namespace cobotsys;

class CobotUrDigitIoAdapter : public AbstractDigitIoDriver {
public:
    CobotUrDigitIoAdapter(CobotUrRealTimeCommCtrl& realTimeCommCtrl);
    virtual ~CobotUrDigitIoAdapter();

    virtual bool setup(const QString& configFilePath);

    virtual void setIo(DigitIoPorts ioPorts, DigitIoStatus ioStatus);
    virtual DigitIoStatus getIoStatus(DigitIoPort ioPort);
    virtual bool isDigitInput() const;
    virtual bool isDigitOutput() const;




    // 这是一个内部类，没有怎么严格的要求写。
    std::vector<DigitIoStatus> m_ioPortStatus;
    bool m_isInput;
    bool m_isOutput;

    int m_inputIoStatus;
    int m_outputIoStatus;

    int m_debugIoLastStatus;

    CobotUrRealTimeCommCtrl& m_realTimeCommCtrl;

    void debugIoStatus();

    void setDigitOut(int portIndex, bool b);

};


#endif //COBOTSYS_COBOTURDIGITIOADAPTER_H
