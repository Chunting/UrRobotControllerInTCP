//
// Created by 潘绪洋 on 17-4-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "RobotManipulator.h"

RobotManipulator::RobotManipulator() {
    ui.setupUi(this);
    m_spinValues.push_back(ui.doubleSpinBox_1);
    m_spinValues.push_back(ui.doubleSpinBox_2);
    m_spinValues.push_back(ui.doubleSpinBox_3);
    m_spinValues.push_back(ui.doubleSpinBox_4);
    m_spinValues.push_back(ui.doubleSpinBox_5);
    m_spinValues.push_back(ui.doubleSpinBox_6);
    ui.rbBase->setChecked(true);

    connect(ui.btnReset, &QPushButton::released, this, &RobotManipulator::onButtonResetJoint);
    connect(ui.btnGo, &QPushButton::released, this, &RobotManipulator::onButtonGo);
}

RobotManipulator::~RobotManipulator() {
    INFO_DESTRUCTOR(this);
}

bool RobotManipulator::setup(const QString& configFilePath) {
    return true;
}

void RobotManipulator::clearAttachedObject() {
    detachSharedObject(m_ptrMover);
    detachSharedObject(m_ptrRobot);
}

void RobotManipulator::setRobotMover(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& robot,
                                     const std::shared_ptr<AbstractArmRobotMoveDriver>& mover) {
    m_ptrRobot = robot;
    m_ptrMover = mover;
}

void RobotManipulator::onButtonResetJoint() {
    if (m_ptrRobot) {
        auto joint = m_ptrRobot->getRobotJointQ();

        joint = convertJoint(joint);

        auto len = smaller(joint.size(), m_spinValues.size());
        for (size_t i = 0; i < len; i++) {
            m_spinValues[i]->setValue(joint[i]);
        }
    }
}

bool RobotManipulator::isBaseUnitType() const {
    return ui.rbBase->isChecked();
}

std::vector<double> RobotManipulator::convertJoint(const std::vector<double>& joint) {
    std::vector<double> retval(joint.size(), 0);
    if (isBaseUnitType()) {
        if (m_ptrMover) {
            auto& solver = m_ptrMover->getKinematicSolver();
            if (solver) {
                solver->jntToCart(joint, retval);
                if (retval.size() == 6) {
                    retval[0] = retval[0] * 1000;
                    retval[1] = retval[1] * 1000;
                    retval[2] = retval[2] * 1000;
                    retval[3] = retval[3] / M_PI * 180;
                    retval[4] = retval[4] / M_PI * 180;
                    retval[5] = retval[5] / M_PI * 180;
                }
            }
        }
    } else {
        for (size_t i = 0; i < joint.size(); i++) {
            retval[i] = joint[i] * 180 / M_PI;
        }
    }

    return retval;
}

void RobotManipulator::onButtonGo() {
    auto vals = getUiTargetValue();
    if (isBaseUnitType()) {
        if (m_ptrMover && (vals.size() == 6)) {
            cv::Point3d pt = {vals[0], vals[1], vals[2]};
            cv::Vec3d rpy = {vals[3], vals[4], vals[5]};;
            m_ptrMover->move(m_ptrMover->generateMoveId(), pt, rpy);
        }
    } else {
        if (m_ptrRobot) {
            m_ptrRobot->move(vals);
        }
    }
}

std::vector<double> RobotManipulator::getUiTargetValue() const {
    std::vector<double> retval(m_spinValues.size());
    for (size_t i = 0; i < m_spinValues.size(); i++) {
        retval[i] = m_spinValues[i]->value();
    }
    if (isBaseUnitType()) {
        if (retval.size() == 6) {
            retval[0] = retval[0] / 1000;
            retval[1] = retval[1] / 1000;
            retval[2] = retval[2] / 1000;

            retval[3] = retval[3] * M_PI / 180;
            retval[4] = retval[4] * M_PI / 180;
            retval[5] = retval[5] * M_PI / 180;
        }
    } else {
        for (size_t i = 0; i < retval.size(); i++) {
            retval[i] = retval[i] * M_PI / 180;
        }
    }
    return retval;
}
