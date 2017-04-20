//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DragAppWidget.h"

DragAppWidget::DragAppWidget() {
	m_ptrForceController.reset(new ForceControllerClass());
    ui.setupUi(this);

	    connect(ui.btnStart, &QPushButton::released, this, &DragAppWidget::start);
    connect(ui.btnStop, &QPushButton::released, this, &DragAppWidget::stop);
    connect(ui.btnInit, &QPushButton::released, this, &DragAppWidget::initUiComboList);
    connect(ui.btnCreateArmRobot, &QPushButton::released, this, &DragAppWidget::createArmRobotDriver);
    connect(ui.btnCreateIk, &QPushButton::released, this, &DragAppWidget::createSolver);
	connect(ui.btnCopy, &QPushButton::released, this, &DragAppWidget::copyCurXyzRpy);
	connect(ui.btnGo, &QPushButton::released, this, &DragAppWidget::onGoCommand);

	connect(ui.btnCopyJoint, &QPushButton::released, this, &DragAppWidget::copyJoint);
	connect(ui.btnGoJoint, &QPushButton::released, this, &DragAppWidget::goJoint);

    connect(this, &DragAppWidget::jointValueUpdated, this, &DragAppWidget::onJointUpdate);
	m_timer = new QTimer(this);
	connect(m_timer, &QTimer::timeout, this, &DragAppWidget::onTimer);
	
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

void DragAppWidget::start() {
	if (m_ptrForceController) {
		m_ptrForceController->initialize();
	}
	if (m_ptrRobot) {
		m_ptrRobot->start();
	}
	if (m_ptrForceSensor) {
		m_ptrForceSensor->start();
	}
	if (m_timer) {
		m_timer->start(2);
	}
}

void DragAppWidget::stop() {
	if (m_timer) {
		m_timer->stop();
	}
	if (m_ptrForceSensor) {
		m_ptrForceSensor->stop();
	}
	if (m_ptrRobot) {
		m_ptrRobot->stop();
	}
}


bool DragAppWidget::setup(const QString& configFilePath) {
	initUiComboList();
	createArmRobotDriver();
    return true;
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


void DragAppWidget::createArmRobotDriver() {
	createSolver();
	createForceSensor();
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboArmRobotList->count() == 0) {
        COBOT_LOG.error() << "No robot driver plugin exist!";
        return;
    }

    //QString objConfig = QFileDialog::getOpenFileName(this,
    //    tr("Get Robot Config JSON file ..."),
    //    QString(FileFinder::getPreDefPath().c_str()),
    //    tr("JSON files (*.JSON *.json)"));
	QString objConfig = FileFinder::find("CONFIG/UrRobotConfig/ur3_180_config.json").c_str();
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


void DragAppWidget::createSolver() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboArmRobotList->count() == 0) {
        COBOT_LOG.error() << "No robot driver plugin exist!";
        return;
    }

    //QString objConfig = QFileDialog::getOpenFileName(this,
    //    tr("Get Solver Config JSON file ..."),
    //    QString(FileFinder::getPreDefPath().c_str()),
    //    tr("JSON files (*.JSON *.json)"));
	QString objConfig = FileFinder::find("CONFIG/UrRobotConfig/ur3_180_config.json").c_str();
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

void DragAppWidget::createForceSensor() {
	if (!GlobalObjectFactory::instance()) return;
	//TODO 临时采用不适用文件浏览器的方式。
	//QString objConfig = QFileDialog::getOpenFileName(this,
	//    tr("Get Solver Config JSON file ..."),
	//    QString(FileFinder::getPreDefPath().c_str()),
	//    tr("JSON files (*.JSON *.json)"));
	QString objConfig = FileFinder::find("CONFIG/ForceControlConfig/optoforce_sensor_config.json").c_str();
	if (objConfig.isEmpty()) {
		COBOT_LOG.notice() << "optoforce sensor config is empty, force sensor create fail.";
		return;
	}

	//QStringList obj_info = ui.cboArmIkList->currentData().toStringList();
	//QString factory = obj_info.front();
	//QString typen = obj_info.back();

	auto obj = GlobalObjectFactory::instance()->createObject("OptoForceSensorFactory, Ver 1.0", "OptoForceSensor");
	m_ptrForceSensor = std::dynamic_pointer_cast<AbstractForceSensor>(obj);
	//TOTO 既然一般来说，json文件中已经包含了factory和typen的信息，何不提供一个接口，让createObject的参数可以直接依据json中的信息来创建。
	if (m_ptrForceSensor) {
		auto ob = std::dynamic_pointer_cast<ForceSensorStreamObserver>(shared_from_this());
		m_ptrForceSensor->attach(ob);
		if (m_ptrForceSensor->setup(objConfig)) {
			COBOT_LOG.notice() << "Create Setup force sensor Success";
		}
		else {
			m_ptrForceSensor.reset();
		}
	}

	ui.btnStart->setEnabled(true);
}

void DragAppWidget::onArmRobotConnect() {
	//TODO 这个接口是否可以提供一个初步的实现。然后可以在继承接口时，可以不必实现。
}

void DragAppWidget::onArmRobotDisconnect() {

}

void DragAppWidget::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    auto q_actual_size = ptrRobotStatus->q_actual.size();
    if (q_actual_size != 6) return;

    m_mutex.lock();
    m_jointValues = ptrRobotStatus->q_actual;
    m_mutex.unlock();

    Q_EMIT jointValueUpdated();
}

void DragAppWidget::onJointUpdate() {
    std::vector<double> qactual;
    m_mutex.lock();
    qactual = m_jointValues;
    m_mutex.unlock();

    for (size_t i = 0; i < qactual.size(); i++) {
        m_dsbJointVals[i]->setValue(qactual[i] / M_PI * 180);
    }

    // TODO here forward-kinxxx
	std::vector<double> xyzrpy;
	m_ptrSolver->jntToCart(qactual, xyzrpy);

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
	m_ptrRobot->move(cmd);
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

	m_ptrSolver->cartToJnt(m_jointValues, cmd, jcmd);


    m_ptrRobot->move(jcmd);
}

void DragAppWidget::onForceSensorConnect() {

}
void DragAppWidget::onForceSensorDisconnect() {

}
void DragAppWidget::onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench) {
	
	m_mutex.lock();
	m_forceValues = *ptrWrench;
	m_mutex.unlock();

	// Update to UI
	ui.dsbFx->setValue(m_forceValues.force.x);
	ui.dsbFy->setValue(m_forceValues.force.y);
	ui.dsbFz->setValue(m_forceValues.force.z);
	ui.dsbTx->setValue(m_forceValues.torque.x);
	ui.dsbTy->setValue(m_forceValues.torque.y);
	ui.dsbTz->setValue(m_forceValues.torque.z);
}
void DragAppWidget::onMoveFinish(uint32_t moveId) {

}

void DragAppWidget::onTimer() {
	//2ms执行一次
	//chrono

	static real_T force_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	static real_T gravity_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	static real_T poseOffset_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Eigen::VectorXd currentJoints(6);
	m_mutex.lock();
	force_ee[0] = m_forceValues.force.x;
	force_ee[1] = m_forceValues.force.y;
	force_ee[2] = m_forceValues.force.z;
	force_ee[3] = m_forceValues.torque.x;
	force_ee[4] = m_forceValues.torque.y;
	force_ee[5] = m_forceValues.torque.z;
	for (int i = 0; i < 6; i++) {
		currentJoints(i) = m_jointValues[i];
	}
	m_mutex.unlock();
	//计算gravity_ee
	Wrench m_loadGravity;
	//TODO 修改接口为cv向量表示。
	Eigen::Vector3d gravity_t_ee;//只考虑三个轴向力，不考虑转矩。
	m_ptrSolver->vector_WorldToEE(currentJoints,
		Eigen::Vector3d(m_loadGravity.force.x, m_loadGravity.force.y, m_loadGravity.force.z),
		gravity_t_ee);
	gravity_ee[0] = gravity_t_ee.x();
	gravity_ee[1] = gravity_t_ee.y();
	gravity_ee[2] = gravity_t_ee.z();
	gravity_ee[3] = m_loadGravity.torque.x;
	gravity_ee[4] = m_loadGravity.torque.y;
	gravity_ee[5] = m_loadGravity.torque.z;

	m_ptrForceController->step(force_ee, gravity_ee, poseOffset_ee);
	std::vector<double> pose_ee;
	for (int i = 0; i < 6; i++) {
		pose_ee.push_back(poseOffset_ee[i]);
	}
	std::vector<double> pose_world;
	m_ptrSolver->pose_EEToWorld(currentJoints, pose_ee, pose_world);
	std::vector<double> targetJoint;
	m_ptrSolver->cartToJnt(currentJoints, pose_world, targetJoint);
	m_ptrRobot->move(targetJoint);

}