//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "RobotXyzWidget.h"

RobotXyzWidget::RobotXyzWidget() {
    ui.setupUi(this);


    connect(ui.btnStart, &QPushButton::released, this, &RobotXyzWidget::start);
    connect(ui.btnStop, &QPushButton::released, this, &RobotXyzWidget::stop);
    connect(ui.btnInit, &QPushButton::released, this, &RobotXyzWidget::initUiComboList);
    connect(ui.btnCreateArmRobot, &QPushButton::released, this, &RobotXyzWidget::createArmRobotDriver);
    connect(ui.btnCreateIk, &QPushButton::released, this, &RobotXyzWidget::createSolver);

    connect(this, &RobotXyzWidget::jointValueUpdated, this, &RobotXyzWidget::onJointUpdate);

    m_dsbJointVals.push_back(ui.dsbRealJoint_1);
    m_dsbJointVals.push_back(ui.dsbRealJoint_1);
    m_dsbJointVals.push_back(ui.dsbRealJoint_2);
    m_dsbJointVals.push_back(ui.dsbRealJoint_3);
    m_dsbJointVals.push_back(ui.dsbRealJoint_4);
    m_dsbJointVals.push_back(ui.dsbRealJoint_5);

}

RobotXyzWidget::~RobotXyzWidget() {
    INFO_DESTRUCTOR(this);
}

void RobotXyzWidget::start() {
}

void RobotXyzWidget::stop() {
}


bool RobotXyzWidget::setup(const QString& configFilePath) {
    return true;
}

void RobotXyzWidget::closeEvent(QCloseEvent* event) {
    QWidget::closeEvent(event);
}

void RobotXyzWidget::initUiComboList() {
    if (!GlobalObjectFactory::instance()) return;

    ui.cboArmRobotList->clear();
    ui.cboArmIkList->clear();
    auto factory_names = GlobalObjectFactory::instance()->getFactoryNames();
    for (auto& name : factory_names) {
        auto types = GlobalObjectFactory::instance()->getFactorySupportedNames(name);

        for (auto& type : types) {
            auto obj = GlobalObjectFactory::instance()->createObject(name, type);
            auto robot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
            QStringList data;
            QString text;

            if (robot) {
                text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                data << name.c_str();
                data << type.c_str();
                ui.cboArmRobotList->addItem(text, data);
            }
            auto ikSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
            if (ikSolver) {
                text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                data << name.c_str();
                data << type.c_str();
                ui.cboArmIkList->addItem(text, data);
            }
        }
    }
}


void RobotXyzWidget::createArmRobotDriver() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboArmRobotList->count() == 0) {
        COBOT_LOG.error() << "No robot driver plugin exist!";
        return;
    }

    QString objConfig = QFileDialog::getOpenFileName(this,
        tr("Get Robot Config JSON file ..."),
        QString(FileFinder::getPreDefPath().c_str()),
        tr("JSON files (*.JSON *.json)"));
    if (objConfig.isEmpty()) {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
        return;
    }

    QStringList obj_info = ui.cboArmRobotList->currentData().toStringList();
    QString factory = obj_info.front();
    QString typen = obj_info.back();

    auto obj = GlobalObjectFactory::instance()->createObject(factory, typen);
    m_ptrRobot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
    if (m_ptrRobot) {
        auto ob = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this());
        m_ptrRobot->attach(ob);
        if (m_ptrRobot->setup(objConfig)) {
            COBOT_LOG.notice() << "Create and setup success";

            ui.label->setText("current robot ip: " + m_ptrRobot->getRobotUrl());
        } else {
            m_ptrRobot.reset();
        }
    }

    ui.btnStart->setEnabled(true);
}


void RobotXyzWidget::createSolver() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboArmRobotList->count() == 0) {
        COBOT_LOG.error() << "No robot driver plugin exist!";
        return;
    }

    QString objConfig = QFileDialog::getOpenFileName(this,
        tr("Get Solver Config JSON file ..."),
        QString(FileFinder::getPreDefPath().c_str()),
        tr("JSON files (*.JSON *.json)"));
    if (objConfig.isEmpty()) {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
        return;
    }

    QStringList obj_info = ui.cboArmIkList->currentData().toStringList();
    QString factory = obj_info.front();
    QString typen = obj_info.back();

    auto obj = GlobalObjectFactory::instance()->createObject(factory, typen);
    m_ptrSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
    if (m_ptrSolver) {
        if (m_ptrSolver->setup(objConfig)) {
            COBOT_LOG.notice() << "Create Setup Solver Success";
        } else {
            m_ptrSolver.reset();
        }
    }

    ui.btnStart->setEnabled(true);
}

void RobotXyzWidget::onArmRobotConnect() {
}

void RobotXyzWidget::onArmRobotDisconnect() {

}

void RobotXyzWidget::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    auto q_actual_size = ptrRobotStatus->q_actual.size();
    if (q_actual_size != 6) return;

    m_mutex.lock();
    m_jointValues = ptrRobotStatus->q_actual;
    m_mutex.unlock();

    Q_EMIT jointValueUpdated();
}

void RobotXyzWidget::onJointUpdate() {
    std::vector<double> qactual;
    m_mutex.lock();
    qactual = m_jointValues;
    m_mutex.unlock();

    for (size_t i = 0; i < qactual.size(); i++) {
        m_dsbJointVals[i]->setValue(qactual[i] / M_PI * 180);
    }

    // TODO here forward-kinxxx
    std::vector<double> xyz;

    // Update to UI
    if (xyz.size() == 6) {
        ui.dsbActual_X->setValue(xyz[0]);
        ui.dsbActual_Y->setValue(xyz[1]);
        ui.dsbActual_Z->setValue(xyz[2]);
        ui.dsbActual_R->setValue(xyz[3]);
        ui.dsbActual_P->setValue(xyz[4]);
        ui.dsbActual_y->setValue(xyz[5]);
    }
}

void RobotXyzWidget::onGoCommand() {
    std::vector<double> cmd;
    std::vector<double> jcmd;

    cmd.push_back(ui.dsbCmd_1->value());
    cmd.push_back(ui.dsbCmd_2->value());
    cmd.push_back(ui.dsbCmd_3->value());
    cmd.push_back(ui.dsbCmd_4->value());
    cmd.push_back(ui.dsbCmd_5->value());
    cmd.push_back(ui.dsbCmd_6->value());

    // TODO here calc cmd to joints and then move
    //m_ptrSolver->

    m_ptrRobot->move(jcmd);
}
