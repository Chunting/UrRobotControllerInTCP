//
// Created by 潘绪洋 on 17-3-17.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include <QtWidgets/QDoubleSpinBox>
#include "UrDebuggerWidget.h"

UrDebuggerWidget::UrDebuggerWidget(){
    ui.setupUi(this);
    m_reverseMove = false;
    m_incBase = 1;
    m_doAction = false;
    m_doSliderMove = false;
    connect(this, &UrDebuggerWidget::jointUpdated, this, &UrDebuggerWidget::updateJointValue);

    connect(ui.dsb_1, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_1->setValue((int) v); });
    connect(ui.dsb_2, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_2->setValue((int) v); });
    connect(ui.dsb_3, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_3->setValue((int) v); });
    connect(ui.dsb_4, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_4->setValue((int) v); });
    connect(ui.dsb_5, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_5->setValue((int) v); });
    connect(ui.dsb_6, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            [=](double v){ if (!m_doSliderMove)ui.slider_6->setValue((int) v); });

    connect(ui.btnStart, &QPushButton::released, this, &UrDebuggerWidget::start);
    connect(ui.btnStop, &QPushButton::released, this, &UrDebuggerWidget::stop);
    connect(ui.btnTargetNow, &QPushButton::released, this, &UrDebuggerWidget::curAsTarget);

    std::vector<QSlider*> sliders;
    sliders.push_back(ui.slider_1);
    sliders.push_back(ui.slider_2);
    sliders.push_back(ui.slider_3);
    sliders.push_back(ui.slider_4);
    sliders.push_back(ui.slider_5);
    sliders.push_back(ui.slider_6);

    for (auto& slider : sliders) {
        connect(slider, &QSlider::sliderMoved, this, &UrDebuggerWidget::setMoveTarget);
        connect(slider, &QSlider::sliderPressed, [=](){ m_doSliderMove = true; });
        connect(slider, &QSlider::sliderReleased, [=](){ m_doSliderMove = false; });
    }
}

UrDebuggerWidget::~UrDebuggerWidget(){
    INFO_DESTRUCTOR(this);
    stop();
}

bool UrDebuggerWidget::start(){
    if (m_robotDriver) {
        m_timeLastJU = std::chrono::high_resolution_clock::now();
        return m_robotDriver->start();
    }
    return false;
}

void UrDebuggerWidget::pause(){
    stop();
}

void UrDebuggerWidget::stop(){
    if (m_robotDriver) {
        m_robotDriver->pause();
        m_jointTarget.clear();
    }
}

bool UrDebuggerWidget::setup(const QString& configFilePath){

    QJsonObject json;
    loadJson(json, configFilePath);

    auto robot_factory = json["robot_factory"].toString("UrRobotDriverFactory, Ver 1.0");
    auto robot_type = json["robot_type"].toString("UrAdapter");
    auto pObject = GlobalObjectFactory::instance()->createObject(robot_factory, robot_type);
    m_robotDriver = std::dynamic_pointer_cast<cobotsys::AbstractRobotDriver>(pObject);

    m_incBase = json["step_angle"].toDouble(1);
    m_singleMoveRangeLow = json["range_low"].toDouble(-180) / 180 * CV_PI;
    m_singleMoveRangeHigh = json["range_high"].toDouble(180) / 180 * CV_PI;

    if (m_singleMoveRangeHigh > CV_PI)
        m_singleMoveRangeHigh = CV_PI;
    if (m_singleMoveRangeLow < -CV_PI)
        m_singleMoveRangeLow = -CV_PI;

    if (m_robotDriver) {
        m_robotDriver->attach(std::dynamic_pointer_cast<RobotStatusObserver>(shared_from_this()));
        m_robotDriver->setup(configFilePath);
    }
    return true;
}

void UrDebuggerWidget::onMoveFinish(uint32_t moveId){
}

void UrDebuggerWidget::onJointStatusUpdate(const std::vector<double>& jointPose){
    m_mutex.lock();
    m_jointStatus = jointPose;
    auto timeCur = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> timeDiff = timeCur - m_timeLastJU;
    m_timeLastJU = timeCur;
    m_timeArray.push_back(timeDiff.count());
    m_mutex.unlock();

    Q_EMIT jointUpdated();

    bool noMove = true;
    if (m_robotDriver) {
        if (m_robotDriver->isReady()) {
            if (m_jointTarget.size()) {
                noMove = false;
                m_robotDriver->move(0, m_jointTarget);
            }
        }
    }
    if (noMove) {
        m_robotDriver->move(0, jointPose);
    }
}

void UrDebuggerWidget::onRobotConnected(std::shared_ptr<AbstractRobotDriver> pRobot){
    m_doAction = true;
}

void UrDebuggerWidget::onRobotDisconnected(std::shared_ptr<AbstractRobotDriver> pRobot){
    m_doAction = false;
}

void UrDebuggerWidget::closeEvent(QCloseEvent* event){
    stop();
    QWidget::closeEvent(event);
}

void UrDebuggerWidget::updateJointValue(){
    std::vector<double> jtmp;
    m_mutex.lock();
    jtmp = m_jointStatus;
    m_mutex.unlock();

    if (jtmp.size() > 0) ui.dsb_1->setValue(jtmp[0] * 180 / CV_PI);
    if (jtmp.size() > 1) ui.dsb_2->setValue(jtmp[1] * 180 / CV_PI);
    if (jtmp.size() > 2) ui.dsb_3->setValue(jtmp[2] * 180 / CV_PI);
    if (jtmp.size() > 3) ui.dsb_4->setValue(jtmp[3] * 180 / CV_PI);
    if (jtmp.size() > 4) ui.dsb_5->setValue(jtmp[4] * 180 / CV_PI);
    if (jtmp.size() > 5) ui.dsb_6->setValue(jtmp[5] * 180 / CV_PI);

    if (m_timeArray.size() > 50)
        m_timeArray.pop_front();

    auto timeAvg = std::accumulate(m_timeArray.begin(), m_timeArray.end(), 0.0) / m_timeArray.size();
    auto freqAvg = 1.0 / timeAvg;
    ui.label_FPS->setText(QString::number(freqAvg));
}

void UrDebuggerWidget::debugMove(int jointId){
    if (m_robotDriver == nullptr) return;
    if (m_doAction && m_robotDriver->isReady()) {
        auto new_joint = m_jointStatus;

        auto move_unit = m_incBase / 180 * CV_PI;
        if (m_reverseMove)
            move_unit *= -1;

        new_joint[jointId] += move_unit;
        if (new_joint[jointId] > m_singleMoveRangeHigh) {
            new_joint[jointId] = m_singleMoveRangeHigh;
            m_reverseMove = true;
        } else if (new_joint[jointId] < m_singleMoveRangeLow) {
            new_joint[jointId] = m_singleMoveRangeLow;
            m_reverseMove = false;
        }

        m_robotDriver->move(0, new_joint);
    }
}

void UrDebuggerWidget::setMoveTarget(int){
    std::vector<double> target;
    target.push_back(ui.slider_1->value());
    target.push_back(ui.slider_2->value());
    target.push_back(ui.slider_3->value());
    target.push_back(ui.slider_4->value());
    target.push_back(ui.slider_5->value());
    target.push_back(ui.slider_6->value());

//    COBOT_LOG.info() << "target: "
//                     << target[0] << ", "
//                     << target[1] << ", "
//                     << target[2] << ", "
//                     << target[3] << ", "
//                     << target[4] << ", "
//                     << target[5];

    for (auto& iter : target)
        iter = iter / 180 * CV_PI;
    m_jointTarget = target;
}

void UrDebuggerWidget::curAsTarget(){
    m_mutex.lock();
    m_jointTarget = m_jointStatus;
    m_mutex.unlock();
}

bool UrDebuggerWidget::moveTarget(){
    if (m_robotDriver) {
        if (m_robotDriver->isReady()) {
            auto curCopy = m_jointStatus;
            if (m_jointTarget.size() && m_jointTarget.size() == curCopy.size()) {

                for (size_t i = 0; i < curCopy.size(); i++) {
                    curCopy[i] = m_jointTarget[i] - curCopy[i];
                    range_limit(curCopy[i], -m_incBase, m_incBase);
                }
                m_robotDriver->move(0, curCopy);
                return true;
            }
        }
    }
    return false;
}
