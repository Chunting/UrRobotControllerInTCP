//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRAGAPPWIDGET_H
#define PROJECT_DRAGAPPWIDGET_H


#include <extra2.h>

#include "ui_DragAppWidget.h"
#include "DragController.h"
//#include <cobotsys_global_object_factory.h>
//#include <cobotsys_file_finder.h>

#include <cobotsys_abstract_widget.h>

//TODO 做一个头文件，可以一次性包含所有的抽象类接口头文件。

#include <QDoubleSpinBox>
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
    void onJointUpdated(const StdVector &joints);
    void onPoseUpdated(const StdVector &xyzrpy);
    void onForceUpdated(const MyWrench &ptrWrench);

protected:
    virtual void closeEvent(QCloseEvent* event);
public:
    virtual bool setup(const QString& configFilePath);


protected:
    void onGoCommand();

protected:
    Ui::DragAppWidget ui;
    std::vector<QDoubleSpinBox*> m_dsbJointVals;
	std::shared_ptr<DragController> m_dragController;
};


#endif //PROJECT_DRAGAPPWIDGET_H
