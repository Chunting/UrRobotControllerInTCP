//
// Created by eleven on 17-4-14.
//

#include "SuckerBinpickingPlacer.h"
#include "cobotsys_logger.h"

SuckerBinpickingPlacer::SuckerBinpickingPlacer()
        :m_digitIoDriver(nullptr)
{

}

void SuckerBinpickingPlacer::placeObject()
{
    if (!m_digitIoDriver) {
        COBOT_LOG.error() << "The digitIoDriver is NULL";
        return;
    }
    
//    m_digitIoDriver->resetIo(DigitIoPort::Port_1);//
}