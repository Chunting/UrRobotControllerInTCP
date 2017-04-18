//
// Created by eleven on 17-4-14.
//

#include "SuckerBinpickingPicker.h"
#include "cobotsys_abstract_digit_io_driver.h"
#include "cobotsys_logger.h"

SuckerBinpickingPicker::SuckerBinpickingPicker()
        :m_digitIoDriver(nullptr)
{

}

void SuckerBinpickingPicker::pickObject(const binpicking::BinObjGrabPose& binObjGrabPose)
{
    if (!m_digitIoDriver) {
        COBOT_LOG.error() << "The digitIoDriver is NULL";
        return;
    }

//    m_digitIoDriver->setIo(DigitIoPort::Port_1);
}