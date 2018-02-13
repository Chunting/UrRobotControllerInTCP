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
    m_suckerPortIndex = DigitIoPort::Port_3;
    m_cylinderPortIndex = DigitIoPort::Port_0;
    m_hasPickDefPose = false;
}

SuckerBinpickingPicker::~SuckerBinpickingPicker() {
}

bool SuckerBinpickingPicker::pickObject(const binpicking::BinObjGrabPose &binObjGrabPose) {
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
    count++;

    auto time_start = std::chrono::high_resolution_clock::now();

    m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Reset);
    m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Reset);

    TargetPoint.clear();

//    m_defHomePos = {0.72928,-0.20981,0.42024};
//    m_defHomeRpy = {-M_PI,0,-M_PI/2};
//    m_defWaitPos = {0.38508,-0.77732,0.37024};
//    m_defWaitRpy = {-M_PI,0,-M_PI/2};
//    m_defPlacePos = {0.43963,-0.29184,-0.26388};
//    m_defPlaceRpy = {-M_PI,0,-M_PI/2};
    m_defHomePos = {-0.72928,0.20981,0.42024};
    m_defHomeRpy = {-M_PI,0,-M_PI/2};
    m_defWaitPos = {-0.38508,0.77732,0.37024};
    m_defWaitRpy = {-M_PI,0,-M_PI/2};
    m_defPlacePos = {-0.43963,0.29184,-0.26388};
    m_defPlaceRpy = {-M_PI,0,-M_PI/2};

    binpicking::BinObjGrabPose positionGrab;
    positionGrab.position.x = -binObjGrabPose.position.x;
    positionGrab.position.y = -binObjGrabPose.position.y;
    positionGrab.position.z = binObjGrabPose.position.z;
    positionGrab.rotation[0] = -CV_PI;
    positionGrab.rotation[1] = 0;
    positionGrab.rotation[2] = -CV_PI / 2;


    positionGrab.position = {-0.38508,0.77732,0.07024};


    m_moveId1 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId1;
    m_targets.position = m_defHomePos;
    m_targets.rpy = m_defHomeRpy;
    TargetPoint.push_back(m_targets);

    m_moveId2 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId2;
    m_targets.position = m_defWaitPos;
    m_targets.rpy = m_defWaitRpy;
    TargetPoint.push_back(m_targets);

    m_moveId3 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId3;
    m_targets.position = positionGrab.position;
    m_targets.rpy = positionGrab.rotation;
    TargetPoint.push_back(m_targets);

    m_moveId4 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId4;
    m_targets.position = m_defWaitPos;
    m_targets.rpy = m_defWaitRpy;
    TargetPoint.push_back(m_targets);

    m_moveId5 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId5;
    m_targets.position = m_defHomePos;
    m_targets.rpy = m_defHomeRpy;
    TargetPoint.push_back(m_targets);

    m_moveId6 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId6;
    m_targets.position = m_defPlacePos;
    m_targets.rpy = m_defPlaceRpy;
    TargetPoint.push_back(m_targets);

    m_moveId7 = m_ptrMover->generateMoveId();
    m_targets.moveId = m_moveId7;
    m_targets.position = m_defHomePos;
    m_targets.rpy = m_defHomeRpy;
    TargetPoint.push_back(m_targets);

    m_ptrMover->move(TargetPoint);

    while(true) {
        m_msg.wait(uniqueLock);
     //   COBOT_LOG.notice("111111111111111111111111111111111111111111111aaa") << m_moveId;
        if(m_moveId == m_moveId2){
            m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Set);
            m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Set);
        }
        if(m_moveId == m_moveId3){
            m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Reset);
        }
        if(m_moveId == m_moveId6){
            m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Set);
            m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Reset);
        }
        if (m_moveId == m_moveId7) {
            m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Reset);

            auto time_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> time_diff =time_end -time_start;
            COBOT_LOG.info() << "******count:"<< count <<"*********************time:" << time_diff.count() * 1000 << "ms";

            break;
        }
    }
    

/*


    m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Reset);
    m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Reset);
//  m_moveResult = MoveResult::Cancled;

    binpicking::BinObjGrabPose positionHome, positionWait, positionPlace, positionGrab, positionStandby;
    positionHome.position.x = 0.72928;
    positionHome.position.y = -0.20981;
    positionHome.position.z = 0.42024;
    positionHome.rotation[0] = -CV_PI;
    positionHome.rotation[1] = 0;
    positionHome.rotation[2] = -CV_PI / 2;

    positionWait.position.x = 0.38508;
    positionWait.position.y = -0.77732;
    positionWait.position.z = 0.42024;
    positionWait.rotation[0] = -CV_PI;
    positionWait.rotation[1] = 0;
    positionWait.rotation[2] = -CV_PI / 2;



    positionPlace.position.x = 0.43963;
    positionPlace.position.y = -0.29184;
    positionPlace.position.z = -0.26388;
    positionPlace.rotation[0] = -CV_PI;
    positionPlace.rotation[1] = 0;
    positionPlace.rotation[2] = -CV_PI / 2;


//    positionGrab.position.x = 0.263;
//    positionGrab.position.y = -0.894;
//    positionGrab.position.z = 0.011;
    positionGrab.position.x = -binObjGrabPose.position.x;
    positionGrab.position.y = -binObjGrabPose.position.y;
    positionGrab.position.z = binObjGrabPose.position.z;
    positionGrab.rotation[0] = -CV_PI;
    positionGrab.rotation[1] = 0;
    positionGrab.rotation[2] = -CV_PI / 2;

    positionStandby.position.x = -binObjGrabPose.position.x;
    positionStandby.position.y = -binObjGrabPose.position.y ;
    positionStandby.position.z = 0.37024;
    positionStandby.rotation[0] = -CV_PI;
    positionStandby.rotation[1] = 0;
    positionStandby.rotation[2] = -CV_PI / 2;


    m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionHome.position, positionHome.rotation);
    m_msg.wait(uniqueLock);

//    m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionWait.position, positionWait.rotation);
//    m_msg.wait(uniqueLock);


    m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionStandby.position, positionStandby.rotation);
    m_msg.wait(uniqueLock);

    if (m_moveResult == MoveResult::Success) {
        m_moveResult = MoveResult::Cancled;

        m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Set);
        m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Set);

        COBOT_LOG.notice() << "Begin standby to grap posi **************************************************";
        m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionGrab.position, positionGrab.rotation);
        m_msg.wait(uniqueLock);

        COBOT_LOG.notice() << "success to grap posi **************************************************";

        //std::this_thread::sleep_for(std::chrono::milliseconds(500));
        if(m_moveResult == MoveResult::Success){

            m_moveResult = MoveResult::Cancled;
            COBOT_LOG.warning() << "**************************************************";
            m_digitIoDriver->setIo(m_cylinderPortIndex, DigitIoStatus::Reset);

            m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionWait.position, positionWait.rotation);
            m_msg.wait(uniqueLock);

            m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionHome.position, positionHome.rotation);
            m_msg.wait(uniqueLock);

            m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionPlace.position, positionPlace.rotation);
            m_msg.wait(uniqueLock);
            if (m_moveResult == MoveResult::Success) {
                m_digitIoDriver->setIo(m_suckerPortIndex, DigitIoStatus::Reset);
            }
            m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionHome.position, positionHome.rotation);
            m_msg.wait(uniqueLock);

        } else{
            COBOT_LOG.warning() << "positionGrab Can Not Reached,return to positionHome";
            m_ptrMover->move((m_moveId = m_ptrMover->generateMoveId()), positionHome.position, positionHome.rotation);
            m_msg.wait(uniqueLock);
        }
    } else{

         COBOT_LOG.warning() << "positionStandby Can Not Reached";
    }


*/

    return true;
}

void SuckerBinpickingPicker::clearAttachedObject() {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    m_msg.notify_all();
    detachSharedObject(m_digitIoDriver);
    detachSharedObject(m_ptrMover);
}

void SuckerBinpickingPicker::onMoveFinish(uint32_t moveId, MoveResult moveResult) {
    std::lock_guard<std::mutex> lockGuard(m_mutex);

    if (moveId == m_moveId1 ||
            moveId == m_moveId2 ||
            moveId == m_moveId3 ||
            moveId == m_moveId4 ||
            moveId == m_moveId5 ||
            moveId == m_moveId6 ||
            moveId == m_moveId7 ) {
        m_moveId = moveId;
        m_moveResult = moveResult;
        m_msg.notify_all();
    }
}

bool SuckerBinpickingPicker::setup(const QString &configFilePath) {
    QJsonObject jsonObject;
    QJsonArray jpos, jrpy;
    if (loadJson(jsonObject, configFilePath)) {
        COBOT_LOG.notice() << "configFilePath: " << configFilePath;
        //m_suckerPortIndex = (DigitIoPort) (1UL << jsonObject["IoPort"].toInt(0));
        if (jsonObject.contains("Home")) {

            m_hasPickDefPose = true;

            jpos = jsonObject["Home"].toObject()["Pos"].toArray();
            jrpy = jsonObject["Home"].toObject()["Rpy"].toArray();
            m_defHomePos.x = jpos[0].toDouble();
            m_defHomePos.y = jpos[1].toDouble();
            m_defHomePos.z = jpos[2].toDouble();
            m_defHomeRpy[0] = jrpy[0].toDouble() * M_PI / 180;
            m_defHomeRpy[1] = jrpy[1].toDouble() * M_PI / 180;
            m_defHomeRpy[2] = jrpy[2].toDouble() * M_PI / 180;
            COBOT_LOG.notice() << "Home: " << m_defHomePos << ", " << m_defHomeRpy;
            m_hasPickDefPose = true;

            jpos = jsonObject["Wait"].toObject()["Pos"].toArray();
            jrpy = jsonObject["Wait"].toObject()["Rpy"].toArray();
            m_defWaitPos.x = jpos[0].toDouble();
            m_defWaitPos.y = jpos[1].toDouble();
            m_defWaitPos.z = jpos[2].toDouble();
            m_defWaitRpy[0] = jrpy[0].toDouble() * M_PI / 180;
            m_defWaitRpy[1] = jrpy[1].toDouble() * M_PI / 180;
            m_defWaitRpy[2] = jrpy[2].toDouble() * M_PI / 180;
            COBOT_LOG.notice() << "Wait: " << m_defWaitPos << ", " << m_defWaitRpy;

            jpos = jsonObject["Place"].toObject()["Pos"].toArray();
            jrpy = jsonObject["Place"].toObject()["Rpy"].toArray();
            m_defPlacePos.x = jpos[0].toDouble();
            m_defPlacePos.y = jpos[1].toDouble();
            m_defPlacePos.z = jpos[2].toDouble();
            m_defPlaceRpy[0] = jrpy[0].toDouble() * M_PI / 180;
            m_defPlaceRpy[1] = jrpy[1].toDouble() * M_PI / 180;
            m_defPlaceRpy[2] = jrpy[2].toDouble() * M_PI / 180;
            COBOT_LOG.notice() << "Place: " << m_defPlacePos << ", " << m_defPlaceRpy;
        }
        return true;
    }
    return false;
}

