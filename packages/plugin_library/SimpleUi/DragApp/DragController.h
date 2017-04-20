//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_DRAGCONTROLLER_H
#define PROJECT_DRAGCONTROLLER_H


#include <extra2.h>

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
#include <QThread>
//TODO 或可以提出去到XApplication
using namespace cobotsys;
typedef std::vector<double> StdVector;
typedef Wrench MyWrench;

class DragController :
	public QThread,
	public AbstractObject,
	public ArmRobotRealTimeStatusObserver,
	public ForceSensorStreamObserver{
    Q_OBJECT
public:
	DragController();
    virtual ~DragController();

    bool createArmRobotDriver(const QString& configFilePath);
    bool createSolver(const QString& configFilePath);
    bool createForceSensor();

    virtual bool setup(const QString& configFilePath);
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

    virtual void onForceSensorConnect();
    virtual void onForceSensorDisconnect();
    virtual void onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench);

Q_SIGNALS:
    void jointUpdated(const StdVector &joints);
    void poseUpdated(const StdVector &xyzrpy);
    void forceUpdated(const MyWrench $ptrWrench);
public Q_SLOTS:
    void onStartDrag();
    void onStopDrag();

protected:
	void run();
	void onMoveFinish(uint32_t moveId);

protected:

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
    std::shared_ptr<AbstractKinematicSolver> m_ptrSolver;
	std::shared_ptr<AbstractForceSensor> m_ptrForceSensor;
	std::shared_ptr<ForceControllerClass> m_ptrForceController;

	QMutex m_mutex;
    bool m_stop;
    std::vector<double> m_jointValues;
	Wrench m_forceValues;
	Wrench m_loadGravity;
};


#endif //PROJECT_DRAGAPPWIDGET_H
