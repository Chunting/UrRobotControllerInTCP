//
// Created by 潘绪洋 on 17-3-27.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include "ArmRobotManipulator.h"
#include <QDebug>
#include <QtWidgets/QFileDialog>
#include <cobotsys_file_finder.h>

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
    connect(ui.btnCreate, &QPushButton::released, this, &ArmRobotManipulator::createRobot);
}

ArmRobotManipulator::~ArmRobotManipulator(){
}

bool ArmRobotManipulator::setup(const QString& configFilePath){
    QJsonObject json;
    if (loadJson(json, configFilePath)) {
        m_joint_num = json["joint_num"].toDouble(6);
        m_defaultRobotInfo.clear();
        m_defaultRobotInfo << json["default_robot"].toObject()["factory"].toString();
        m_defaultRobotInfo << json["default_robot"].toObject()["type"].toString();
        qDebug() << m_defaultRobotInfo;

        setupCreationList();
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
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboRobotType->count() == 0)
        return;

    QStringList obj_info = ui.cboRobotType->currentData().toStringList();
    QString factory = obj_info.front();
    QString typen = obj_info.back();

    auto obj = GlobalObjectFactory::instance()->createObject(factory, typen);
    m_ptrRobot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
    if (m_ptrRobot) {
        QString robotConfig = QFileDialog::getOpenFileName(this,
                                                           tr("Get Robot Config JSON file ..."),
                                                           QString(FileFinder::getPreDefPath().c_str()),
                                                           tr("JSON files (*.JSON *.json)"));
        if (robotConfig.size() && m_ptrRobot->setup(robotConfig)) {
            COBOT_LOG.notice() << "Create and setup success";
        } else {
            m_ptrRobot.reset();
        }
    }
}

void ArmRobotManipulator::setupCreationList(){
    if (!GlobalObjectFactory::instance()) return;

    bool foundDefault = false;
    QString defIdxText;

    ui.cboRobotType->clear();
    auto factory_names = GlobalObjectFactory::instance()->getFactoryNames();
    for (auto& name : factory_names) {
        auto types = GlobalObjectFactory::instance()->getFactorySupportedNames(name);

        for (auto& type : types) {
            auto obj = GlobalObjectFactory::instance()->createObject(name, type);
            auto robot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
            if (robot) {
                QStringList data;
                QString text;
                text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                data << name.c_str();
                data << type.c_str();
                ui.cboRobotType->addItem(text, data);
                if (data == m_defaultRobotInfo) {
                    foundDefault = true;
                    defIdxText = text;
                }
            }
        }
    }

    if (foundDefault) {
        auto defaultIndex = ui.cboRobotType->findText(defIdxText);
        ui.cboRobotType->setCurrentIndex(defaultIndex);
        COBOT_LOG.info() << "Default ROBOT: " << defaultIndex;
    }
}
