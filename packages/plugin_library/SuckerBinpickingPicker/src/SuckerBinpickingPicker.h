//
// Created by eleven on 17-4-14.
//

#ifndef COBOTSYS_SUCKERBINPICKINGPICKER_H
#define COBOTSYS_SUCKERBINPICKINGPICKER_H

#include "cobotsys_abstract_binpicking_picker.h"
#include "cobotsys_abstract_arm_robot_move_driver.h"
#include <mutex>
#include <condition_variable>

using namespace cobotsys;

class SuckerBinpickingPicker :
        public binpicking::AbstractBinpickingPicker,
        public ArmRobotMoveStatusObserver {
public:
    SuckerBinpickingPicker();
    virtual ~SuckerBinpickingPicker();

    virtual bool pickObject(const binpicking::BinObjGrabPose& binObjGrabPose);

    virtual bool setup(const QString& configFilePath);

    virtual void setRobotDriver(std::shared_ptr<AbstractArmRobotMoveDriver> robotDriver) {
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        m_ptrMover = robotDriver;
        m_ptrMover->attach(std::dynamic_pointer_cast<ArmRobotMoveStatusObserver>(shared_from_this()));
    };

    virtual void setDigitIoDriver(std::shared_ptr<AbstractDigitIoDriver> digitIoDriver) {
        std::lock_guard<std::mutex> lockGuard(m_mutex);
        m_digitIoDriver = digitIoDriver;
    };
    virtual void clearAttachedObject();
    virtual void onMoveFinish(uint32_t moveId, MoveResult moveResult);
private:
    std::shared_ptr<AbstractDigitIoDriver> m_digitIoDriver;
    std::shared_ptr<AbstractArmRobotMoveDriver> m_ptrMover;
    std::mutex m_mutex;
    std::condition_variable m_msg;
    uint32_t m_moveId;
    MoveResult m_moveResult;
    DigitIoPort m_suckerPortIndex;
    DigitIoPort m_cylinderPortIndex;
    bool m_hasPickDefPose;

    cv::Point3d m_defHomePos;
    cv::Vec3d m_defHomeRpy;
    cv::Point3d m_defWaitPos;
    cv::Vec3d m_defWaitRpy;
    cv::Point3d m_defPlacePos;
    cv::Vec3d m_defPlaceRpy;

    std::vector<RobotWaypoint> TargetPoint;
    RobotWaypoint m_targets;
    uint32_t m_moveId1;
    uint32_t m_moveId2;
    uint32_t m_moveId3;
    uint32_t m_moveId4;
    uint32_t m_moveId5;
    uint32_t m_moveId6;
    uint32_t m_moveId7;
    unsigned int count;


};


#endif //COBOTSYS_SUCKERBINPICKINGPICKER_H
