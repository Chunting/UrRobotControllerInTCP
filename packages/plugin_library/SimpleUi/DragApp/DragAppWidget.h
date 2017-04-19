//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRAGAPPWIDGET_H
#define PROJECT_DRAGAPPWIDGET_H


#include <extra2.h>

#include "ui_DragAppWidget.h"
#include "ForceController.h"

#include <cobotsys_global_object_factory.h>
#include <cobotsys_file_finder.h>

#include <cobotsys_abstract_widget.h>
#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_kinematic_solver.h>
#include <cobotsys_abstract_force_sensor.h>

//TODO 做一个头文件，可以一次性包含所有的抽象类接口头文件。

#include <QFileDialog>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QTimer>
#include <QMutex>
//TODO 或可以提出去到XApplication
using namespace cobotsys;

class DragAppWidget :
	public AbstractWidget,
	public ArmRobotMoveStatusObserver,
	public ForceSensorStreamObserver{
    Q_OBJECT
public:
	DragAppWidget();
    virtual ~DragAppWidget();

    void start();
    void stop();


    void initUiComboList();

    void createArmRobotDriver();
    void createSolver();
	void createForceSensor();
	void copyCurXyzRpy();

	void copyJoint();
	void goJoint();

Q_SIGNALS:
    void jointValueUpdated();

protected:
    virtual void closeEvent(QCloseEvent* event);
public:
    virtual bool setup(const QString& configFilePath);


public:
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

	virtual void onForceSensorConnect();
	virtual void onForceSensorDisconnect();
	virtual void onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench);

protected:
    void onJointUpdate();
    void onGoCommand();
	void onTimer();

	void onMoveFinish(uint32_t moveId);
protected:
    Ui::DragAppWidget ui;
    std::vector<QDoubleSpinBox*> m_dsbJointVals;

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
    std::shared_ptr<AbstractKinematicSolver> m_ptrSolver;
	std::shared_ptr<AbstractForceSensor> m_ptrForceSensor;
	std::shared_ptr<ForceControllerClass> m_ptrForceController;

	QTimer *m_timer;
	QMutex m_mutex;

    std::vector<double> m_jointValues;
	Wrench m_forceValues;
	Wrench m_loadGravity;
};


#endif //PROJECT_DRAGAPPWIDGET_H
