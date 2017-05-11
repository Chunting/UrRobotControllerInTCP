//
// Created by 杨帆 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_POLISHAPPWIDGET_H
#define PROJECT_POLISHAPPWIDGET_H


#include <extra2.h>

//#include "ForceController.h"

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

#include "polishingtask.h"
#include "ForceController.h"
//TODO 或可以提出去到XApplication
using namespace cobotsys;
typedef std::vector<double> StdVector;
typedef Wrench MyWrench;

class PolishController :
	public QThread,
	public AbstractObject,
	public ArmRobotRealTimeStatusObserver,
	public ForceSensorStreamObserver{
    Q_OBJECT
public:
	PolishController();
    virtual ~PolishController();
	enum Controller_Status{IDLE,CALIB,DRAG};
    bool createArmRobotDriver(const QString& configFilePath);
    bool createSolver(const QString& configFilePath);
    bool createForceSensor();
	void setControllerStatus(PolishController::Controller_Status status);
    virtual bool setup(const QString& configFilePath);
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

    virtual void onForceSensorConnect();
    virtual void onForceSensorDisconnect();
    virtual void onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench);
	Eigen::Vector3d gravity_t_ee;//只考虑三个轴向力，不考虑转矩。
	double force_ee[6];
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
	void PolishPhase();
	void GravityCalib();
	double parseFzTouch();
	std::vector<double>  MotionPlanner(double disOffset);
	void cartesian_path_generator();
	KDL::Frame getPolisherFrame(int id);
	void defineInterimPath();
	bool checkPathContinuity(int path_index);
	std::vector<KDL::Frame> planInterimPath(int path_index);
protected:

    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
    std::shared_ptr<AbstractKinematicSolver> m_ptrSolver;
	std::shared_ptr<AbstractForceSensor> m_ptrForceSensor;
	std::shared_ptr<ForceControllerClass> m_ptrForceController;

	QMutex m_mutex;
    bool m_stop;
    std::vector<double> m_jointValues;
	Wrench m_forceValues;
	Wrench m_Optoforce_Offset;
	Wrench m_loadGravity;
	Controller_Status m_controllerStatus;
	std::string m_model_path;
	std::string m_ptd_path;
	PolishingTask m_polishingTask;
    bool force_control_en;
	std::vector<std::vector<KDL::Frame> > cartesian_path_;
	std::vector<std::vector<KDL::Frame> > interim_path_;
};


#endif //PROJECT_POLISHAPPWIDGET_H
