//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <extra2.h>
#include "ArmRobotManipulator.h"

class AutoCtx {
protected:
    bool m_sign;
public:
    AutoCtx(bool& b) : m_sign(b){ m_sign = true; }

    ~AutoCtx(){ m_sign = false; }
};
#define Q_SLOT_REDUCE(flag) if ((flag)) {return;} AutoCtx autoctx(flag);


ArmRobotManipulator::ArmRobotManipulator(){
    ui.setupUi(this);

    m_noHandleChange = false;

    m_sliders.push_back(ui.slider_1);
    m_sliders.push_back(ui.slider_2);
    m_sliders.push_back(ui.slider_3);
    m_sliders.push_back(ui.slider_4);
    m_sliders.push_back(ui.slider_5);
    m_sliders.push_back(ui.slider_6);

    m_target.push_back(ui.dsb_target_1);
    m_target.push_back(ui.dsb_target_2);
    m_target.push_back(ui.dsb_target_3);
    m_target.push_back(ui.dsb_target_4);
    m_target.push_back(ui.dsb_target_5);
    m_target.push_back(ui.dsb_target_6);

    m_actual.push_back(ui.dsb_real_1);
    m_actual.push_back(ui.dsb_real_2);
    m_actual.push_back(ui.dsb_real_3);
    m_actual.push_back(ui.dsb_real_4);
    m_actual.push_back(ui.dsb_real_5);
    m_actual.push_back(ui.dsb_real_6);

    for (auto& slider : m_sliders) {
        connect(slider, &QSlider::valueChanged, [=](int){ this->handleSliderChange(); });
    }

    for (auto& iter : m_target) {
        connect(iter, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                [=](double){ handleTargetChange(); });
    }
}

ArmRobotManipulator::~ArmRobotManipulator(){
}

bool ArmRobotManipulator::setup(const QString& configFilePath){
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_joint_num = json["joint_num"].toDouble(6);
        return true;
    }
    return false;
}

void ArmRobotManipulator::handleSliderChange(){
    Q_SLOT_REDUCE(m_noHandleChange);

    for (size_t i = 0; i < m_sliders.size(); i++) {
        auto slider = m_sliders[i];
        auto dsbTgt = m_target[i];
        auto dsbAct = m_actual[i];
        if (slider->value() != (int) dsbTgt->value()) {
            dsbTgt->setValue(slider->value());
        }
    }
    m_noHandleChange = false;
}

void ArmRobotManipulator::handleTargetChange(){
    Q_SLOT_REDUCE(m_noHandleChange);

    for (size_t i = 0; i < m_target.size(); i++) {
        auto slider = m_sliders[i];
        auto dsbTgt = m_target[i];
        auto dsbAct = m_actual[i];

        if ((int) dsbTgt->value() != slider->value()) {
            slider->setValue((int) dsbTgt->value());
            dsbTgt->setValue(slider->value());
            return;
        }
    }
}

void ArmRobotManipulator::createRobot(){
}
