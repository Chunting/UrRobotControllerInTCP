//
// Created by eleven on 17-4-14.
//

#ifndef COBOTSYS_SUCKERBINPICKINGPLACER_H
#define COBOTSYS_SUCKERBINPICKINGPLACER_H

#include "cobotsys_abstract_binpicking_placer.h"

using namespace cobotsys;

class SuckerBinpickingPlacer : public AbstractBinpickingPlacer {
public:
    SuckerBinpickingPlacer();

    virtual void placeObject();

    virtual bool setup(const QString& configFilePath) { return true; }
    virtual void setRobotDriver(std::shared_ptr<AbstractArmRobotMoveDriver> robotDriver) { };
    virtual void setDigitIoDriver(std::shared_ptr<AbstractDigitIoDriver> digitIoDriver)
    {
        m_digitIoDriver = digitIoDriver;
    };
private:
    std::shared_ptr<AbstractDigitIoDriver> m_digitIoDriver;
};


#endif //COBOTSYS_SUCKERBINPICKINGPLACER_H
