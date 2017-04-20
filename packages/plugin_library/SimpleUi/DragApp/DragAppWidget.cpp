//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DragAppWidget.h"

DragAppWidget::DragAppWidget() {
    m_dragController=new DragController();
    ui.setupUi(this);
	connect(ui.btnStart, &QPushButton::released, m_dragController, &DragController::onStartDrag);
    connect(ui.btnStop, &QPushButton::released, m_dragController, &DragController::onStopDrag);
    connect(ui.btnInit, &QPushButton::released, this, &DragAppWidget::initUiComboList);
	connect(ui.btnCopy, &QPushButton::released, this, &DragAppWidget::copyCurXyzRpy);
	connect(ui.btnGo, &QPushButton::released, this, &DragAppWidget::onGoCommand);

	connect(ui.btnCopyJoint, &QPushButton::released, this, &DragAppWidget::copyJoint);
	connect(ui.btnGoJoint, &QPushButton::released, this, &DragAppWidget::goJoint);

    connect(m_dragController, &DragController::jointUpdated, this, &DragAppWidget::onJointUpdated);
    connect(m_dragController, &DragController::poseUpdated, this, &DragAppWidget::onPoseUpdated);
    connect(m_dragController, &DragController::forceUpdated, this, &DragAppWidget::onForceUpdated);

	
    m_dsbJointVals.push_back(ui.dsbRealJoint_1);
    m_dsbJointVals.push_back(ui.dsbRealJoint_2);
    m_dsbJointVals.push_back(ui.dsbRealJoint_3);
    m_dsbJointVals.push_back(ui.dsbRealJoint_4);
    m_dsbJointVals.push_back(ui.dsbRealJoint_5);
	m_dsbJointVals.push_back(ui.dsbRealJoint_6);
}

DragAppWidget::~DragAppWidget() {
    INFO_DESTRUCTOR(this);
}


bool DragAppWidget::setup(const QString& configFilePath) {
	initUiComboList();
    return m_dragController->setup(configFilePath);
}

void DragAppWidget::closeEvent(QCloseEvent* event) {
    QWidget::closeEvent(event);
}

void DragAppWidget::initUiComboList() {
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

void DragAppWidget::onJointUpdated(std::vector<double> joints) {
    for (size_t i = 0; i < joints.size(); i++) {
        m_dsbJointVals[i]->setValue(joints[i] / M_PI * 180);
    }
}
void DragAppWidget::onPoseUpdated(std::vector<double> xyzrpy){
    // Update to UI
    if (xyzrpy.size() == 6) {
        ui.dsbActual_X->setValue(xyzrpy[0] * 1000);
        ui.dsbActual_Y->setValue(xyzrpy[1] * 1000);
        ui.dsbActual_Z->setValue(xyzrpy[2] * 1000);
        ui.dsbActual_R->setValue(xyzrpy[3] / M_PI * 180);
        ui.dsbActual_P->setValue(xyzrpy[4] / M_PI * 180);
        ui.dsbActual_y->setValue(xyzrpy[5] / M_PI * 180);
    }
}
void DragAppWidget::onForceUpdated(Wrench& ptrWrench) {
    // Update to UI
    ui.dsbFx->setValue(ptrWrench.force.x);
    ui.dsbFy->setValue(ptrWrench.force.y);
    ui.dsbFz->setValue(ptrWrench.force.z);
    ui.dsbTx->setValue(ptrWrench.torque.x);
    ui.dsbTy->setValue(ptrWrench.torque.y);
    ui.dsbTz->setValue(ptrWrench.torque.z);
}

void DragAppWidget::copyJoint() {
	ui.dsbCmd_1->setValue(ui.dsbRealJoint_1->value());
	ui.dsbCmd_2->setValue(ui.dsbRealJoint_2->value());
	ui.dsbCmd_3->setValue(ui.dsbRealJoint_3->value());
	ui.dsbCmd_4->setValue(ui.dsbRealJoint_4->value());
	ui.dsbCmd_5->setValue(ui.dsbRealJoint_5->value());
	ui.dsbCmd_6->setValue(ui.dsbRealJoint_6->value());
}

void DragAppWidget::goJoint() {
	std::vector<double> cmd;
	cmd.push_back(ui.dsbCmd_1->value() * M_PI / 180);
	cmd.push_back(ui.dsbCmd_2->value() * M_PI / 180);
	cmd.push_back(ui.dsbCmd_3->value() * M_PI / 180);
	cmd.push_back(ui.dsbCmd_4->value() * M_PI / 180);
	cmd.push_back(ui.dsbCmd_5->value() * M_PI / 180);
	cmd.push_back(ui.dsbCmd_6->value() * M_PI / 180);
	//m_ptrRobot->move(cmd);
}

void DragAppWidget::copyCurXyzRpy() {
	ui.dsbCmd_1->setValue(ui.dsbActual_X->value());
	ui.dsbCmd_2->setValue(ui.dsbActual_Y->value());
	ui.dsbCmd_3->setValue(ui.dsbActual_Z->value());
	ui.dsbCmd_4->setValue(ui.dsbActual_R->value());
	ui.dsbCmd_5->setValue(ui.dsbActual_P->value());
	ui.dsbCmd_6->setValue(ui.dsbActual_y->value());
}

void DragAppWidget::onGoCommand() {
    std::vector<double> cmd;
    std::vector<double> jcmd;

    cmd.push_back(ui.dsbCmd_1->value() / 1000);
    cmd.push_back(ui.dsbCmd_2->value() / 1000);
    cmd.push_back(ui.dsbCmd_3->value() / 1000);
	cmd.push_back(ui.dsbCmd_4->value() / 180 * M_PI);
    cmd.push_back(ui.dsbCmd_5->value() / 180 * M_PI);
    cmd.push_back(ui.dsbCmd_6->value() / 180 * M_PI);

	//m_ptrSolver->cartToJnt(m_jointValues, cmd, jcmd);
    //m_ptrRobot->move(jcmd);
}
