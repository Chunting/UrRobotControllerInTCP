//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRAGAPPWIDGET_H
#define PROJECT_DRAGAPPWIDGET_H


#include <extra2.h>

#include "ui_DragAppWidget.h"
#include "DragController.h"
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
	public AbstractWidget{
    Q_OBJECT
public:
	DragAppWidget();
    virtual ~DragAppWidget();

    void initUiComboList();
	void copyCurXyzRpy();
	void copyJoint();
	void goJoint();


public Q_SLOTS:
    void onJointUpdated(std::vector<double> joints);
    void onPoseUpdated(std::vector<double> xyzrpy);
    void onForceUpdated(Wrench& ptrWrench);

protected:
    virtual void closeEvent(QCloseEvent* event);
public:
    virtual bool setup(const QString& configFilePath);


protected:
    void onGoCommand();

protected:
    Ui::DragAppWidget ui;
    std::vector<QDoubleSpinBox*> m_dsbJointVals;
	DragController* m_dragController;
};


#endif //PROJECT_DRAGAPPWIDGET_H
