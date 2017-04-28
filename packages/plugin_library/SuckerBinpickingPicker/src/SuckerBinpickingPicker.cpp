//
// Created by eleven on 17-4-14.
//

#include <QtCore/QJsonObject>
#include <extra2.h>
#include <QtCore/QJsonArray>
#include "SuckerBinpickingPicker.h"
#include "cobotsys_abstract_digit_io_driver.h"
#include "cobotsys_logger.h"

SuckerBinpickingPicker::SuckerBinpickingPicker()
        : m_digitIoDriver(nullptr) {
    m_suckerPortIndex = DigitIoPort::Port_0;
    m_hasPickDefPose = false;
}

bool SuckerBinpickingPicker::pickObject(const binpicking::BinObjGrabPose& binObjGrabPose) {
    std::mutex localmutex;
    std::unique_lock<std::mutex> uniqueLock(localmutex);

    if (!m_digitIoDriver) {
        COBOT_LOG.error() << "The digitIoDriver is NULL";
        return false;
    }

    if (!m_ptrMover) {
        COBOT_LOG.error() << "No Mover";
        return false;
    }

    m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Reset);

    // First move to target
    m_moveId = m_ptrMover->generateMoveId();
    m_ptrMover->move(m_moveId, binObjGrabPose.position, binObjGrabPose.rotation);
    m_msg.wait(uniqueLock);


    // Pick Object
    if (m_moveResult == MoveResult::Success) {
        m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Set);
        COBOT_LOG.debug() << "Move: " << std::setw(3) << m_moveId << " Success";
    } else {
        COBOT_LOG.error() << "Robot Move Fail";
        return false;
    }

    // Move up, if config this option
    if (m_hasPickDefPose) {
        m_moveId = m_ptrMover->generateMoveId();
        m_ptrMover->move(m_moveId, binObjGrabPose.position, binObjGrabPose.rotation);
        m_msg.wait(uniqueLock);

        if (m_moveResult == MoveResult::Success) {
        } else {
            COBOT_LOG.error() << "Robot Move Fail";
            return false;
        }
    }

    return true;
}

void SuckerBinpickingPicker::clearAttachedObject() {
    detachSharedObject(m_digitIoDriver);
    detachSharedObject(m_ptrMover);
}

void SuckerBinpickingPicker::onMoveFinish(uint32_t moveId, MoveResult moveResult) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (moveId == m_moveId) {
        m_moveResult = moveResult;
        m_msg.notify_all();
    }
}

bool SuckerBinpickingPicker::setup(const QString& configFilePath) {
    QJsonObject jsonObject;
    if (loadJson(jsonObject, configFilePath)) {
        m_suckerPortIndex = (DigitIoPort) (1UL << jsonObject["IoPort"].toInt(0));
        if (jsonObject.contains("PickHome")) {
            auto jpos = jsonObject["PickHome"].toObject()["Pos"].toArray();
            auto jrpy = jsonObject["PickHome"].toObject()["Rpy"].toArray();
            m_defHomePos.x = jpos[0].toDouble();
            m_defHomePos.y = jpos[1].toDouble();
            m_defHomePos.z = jpos[2].toDouble();
            m_defHomeRpy[0] = jrpy[0].toDouble();
            m_defHomeRpy[1] = jrpy[1].toDouble();
            m_defHomeRpy[2] = jrpy[2].toDouble();
            m_hasPickDefPose = true;
            COBOT_LOG.notice() << "Pick Home: " << m_defHomePos << ", " << m_defHomeRpy;
        }
        return true;
    }
    return false;
}
