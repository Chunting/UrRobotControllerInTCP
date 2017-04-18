//
// Created by eleven on 17-4-14.
//

#ifndef COBOTSYS_SUCKERBINPICKINGPICKER_H
#define COBOTSYS_SUCKERBINPICKINGPICKER_H

#include "cobotsys_abstract_binpicking_picker.h"

using namespace cobotsys;

class SuckerBinpickingPicker : public binpicking::AbstractBinpickingPicker {
public:
    SuckerBinpickingPicker();

    virtual void pickObject(const binpicking::BinObjGrabPose& binObjGrabPose);

    virtual bool setup(const QString& configFilePath) { return true; }
    virtual void setRobotDriver(std::shared_ptr<AbstractArmRobotMoveDriver> robotDriver) { };
    virtual void setDigitIoDriver(std::shared_ptr<AbstractDigitIoDriver> digitIoDriver)
    {
        m_digitIoDriver = digitIoDriver;
    };
private:
    std::shared_ptr<AbstractDigitIoDriver> m_digitIoDriver;
};


#endif //COBOTSYS_SUCKERBINPICKINGPICKER_H
