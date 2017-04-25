//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DragController.h"
typedef std::chrono::high_resolution_clock::time_point timestamp;


DragController::DragController() :m_stop(false){
	m_ptrForceController.reset(new ForceControllerClass());
	m_jointValues.resize(6);
    m_loadGravity.force=cv::Point3d(0.0,0.0,1);
    m_loadGravity.torque=cv::Point3d(0.0,0.0,0.0);
    m_controllerStatus=IDLE;
    for(int i=0;i<6;i++){
        force_ee[i] = 0.0;
    }
}

DragController::~DragController() {
    INFO_DESTRUCTOR(this);
}

void DragController::onStartDrag() {
	if (m_ptrForceController) {
		m_ptrForceController->initialize();
	}
	if (m_ptrRobot) {
		m_ptrRobot->start();
	}
	if (m_ptrForceSensor) {
		m_ptrForceSensor->start();
	}
    m_stop=false;
    std::vector<double> targetJoint;
    targetJoint.push_back(0.0);
    targetJoint.push_back(0.0);
    targetJoint.push_back(0.0);
    targetJoint.push_back(0.0);
    targetJoint.push_back(0.0);
    targetJoint.push_back(0.0);
    m_ptrRobot->move(targetJoint);
    start();
}

void DragController::onStopDrag() {
    m_stop=true;
    quit();
    if (m_ptrForceSensor) {
		m_ptrForceSensor->stop();
	}
	if (m_ptrRobot) {
		m_ptrRobot->stop();
	}
}


bool DragController::setup(const QString& configFilePath) {
	bool result=true;
	result&=createArmRobotDriver(configFilePath);
	result&=createSolver(configFilePath);
	result&=createForceSensor();
    return result;
}

bool DragController::createArmRobotDriver(const QString& configFilePath) {
    if (!GlobalObjectFactory::instance()) return false;
	QString objConfig = FileFinder::find(configFilePath.toStdString()).c_str();
    if (objConfig.isEmpty()) {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
        return false;
    }
    auto obj = GlobalObjectFactory::instance()->createObject("URRealTimeDriverFactory, Ver 1.0", "URRealTimeDriver");
    m_ptrRobot = std::dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(obj);
    if (m_ptrRobot) {
		auto ob = std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this());
        m_ptrRobot->attach(ob);
        if (m_ptrRobot->setup(objConfig)) {
            COBOT_LOG.notice() << "Create and setup success";
			return true;
        } else {
            m_ptrRobot.reset();
            return false;
        }
    }
    else{
        return false;
    }

}


bool DragController::createSolver(const QString& configFilePath) {
    if (!GlobalObjectFactory::instance()) return false;

	QString objConfig = FileFinder::find(configFilePath.toStdString()).c_str();
    if (objConfig.isEmpty()) {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
        return false;
    }

    auto obj = GlobalObjectFactory::instance()->createObject("KinematicSolverFactory, Ver 1.0", "KinematicSolver");
    m_ptrSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
    if (m_ptrSolver) {
        if (m_ptrSolver->setup(objConfig)) {
            COBOT_LOG.notice() << "Create Setup Solver Success";
			return true;
        } else {
            m_ptrSolver.reset();
            return false;
        }
    }else{
        return false;
    }
}

bool DragController::createForceSensor() {
	if (!GlobalObjectFactory::instance()) return false;
	//TODO 临时采用不适用文件浏览器的方式。

	QString objConfig = FileFinder::find("CONFIG/ForceControlConfig/optoforce_sensor_config.json").c_str();
	if (objConfig.isEmpty()) {
		COBOT_LOG.notice() << "optoforce sensor config is empty, force sensor create fail.";
		return false;
	}

	auto obj = GlobalObjectFactory::instance()->createObject("OptoForceSensorFactory, Ver 1.0", "OptoForceSensor");
	m_ptrForceSensor = std::dynamic_pointer_cast<AbstractForceSensor>(obj);
	//TOTO 既然一般来说，json文件中已经包含了factory和typen的信息，何不提供一个接口，让createObject的参数可以直接依据json中的信息来创建。
	if (m_ptrForceSensor) {
		auto ob = std::dynamic_pointer_cast<ForceSensorStreamObserver>(shared_from_this());
		m_ptrForceSensor->attach(ob);
		if (m_ptrForceSensor->setup(objConfig)) {
			COBOT_LOG.notice() << "Create Setup force sensor Success";
			return true;
		}
		else {
			m_ptrForceSensor.reset();
		}
	}
	return false;
}

void DragController::onArmRobotConnect() {
	//TODO 这个接口是否可以提供一个初步的实现。然后可以在继承接口时，可以不必实现。
}

void DragController::onArmRobotDisconnect() {

}

void DragController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
    auto q_actual_size = ptrRobotStatus->q_actual.size();
    if (q_actual_size != 6) return;

    m_mutex.lock();
    m_jointValues = ptrRobotStatus->q_actual;
    m_mutex.unlock();

    Q_EMIT jointUpdated(m_jointValues);

	std::vector<double> xyzrpy;
	m_ptrSolver->jntToCart(ptrRobotStatus->q_actual, xyzrpy);

	Q_EMIT poseUpdated(xyzrpy);
}

void DragController::onForceSensorConnect() {

}
void DragController::onForceSensorDisconnect() {

}
void DragController::onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench) {

	m_mutex.lock();
	m_forceValues = *ptrWrench;
    force_ee[0] = m_forceValues.force.x-m_Optoforce_Offset.force.x;
    force_ee[1] = m_forceValues.force.y-m_Optoforce_Offset.force.y;
    force_ee[2] = m_forceValues.force.z-m_Optoforce_Offset.force.z;
    force_ee[3] = m_forceValues.torque.x-m_Optoforce_Offset.torque.x;
    force_ee[4] = m_forceValues.torque.y-m_Optoforce_Offset.torque.y;
    force_ee[5] = m_forceValues.torque.z-m_Optoforce_Offset.torque.z;
	m_mutex.unlock();
    Q_EMIT forceUpdated(m_forceValues);
}
void DragController::onMoveFinish(uint32_t moveId) {

}
void DragController::setControllerStatus(DragController::Controller_Status status){
    m_controllerStatus=status;
    COBOT_LOG.notice()<<"the drag controller status is set to "<< (int)status;
}
void DragController::run() {
    bool stopflag=false;

    while(!stopflag){
        switch (m_controllerStatus)
        {
            case IDLE:
                break;
            case CALIB://Gravity Calibration
                GravityCalib();
                break;
            case DRAG://Drag Phase
                DragPhase();
                break;
            default:
                COBOT_LOG.error()<<"Undefined controller status";
                break;
        }
        m_mutex.lock();
        stopflag=m_stop;
        m_mutex.unlock();
    }
}
void DragController::GravityCalib(){
    Eigen::VectorXd currentJoints(6);
    //Wrench m_loadGravity;
    m_mutex.lock();
    for (int i = 0; i < 6; i++) {
        currentJoints(i) = m_jointValues[i];
    }
    m_mutex.unlock();
//    std::vector<double> pose_world;
//    pose_world.clear();
//    pose_world.push_back(200);
//    pose_world.push_back(200);
//    pose_world.push_back(200);
//    pose_world.push_back(0);
//    pose_world.push_back(0);
//    pose_world.push_back(0);
    std::vector<double> targetJoint;
    COBOT_LOG.notice()<<"Start gravity calibaration.";
//    m_ptrSolver->cartToJnt(currentJoints, pose_world, targetJoint);
    targetJoint.clear();
    targetJoint.push_back(0);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(0);
    m_ptrRobot->move(targetJoint);
    COBOT_LOG.notice()<<"Moved to up position.";

    Wrench upForce;
    upForce.force.x = 0;
    upForce.force.y = 0;
    upForce.force.z = 0;
    upForce.torque.x = 0;
    upForce.torque.y = 0;
    upForce.torque.z = 0;
    sleep(3);
    for(int i=0;i<100;i++){
        sleep(0.01);
        m_mutex.lock();
        upForce.force.x += m_forceValues.force.x/100;
        upForce.force.y += m_forceValues.force.y/100;
        upForce.force.z += m_forceValues.force.z/100;
        upForce.torque.x += m_forceValues.torque.x/100;
        upForce.torque.y += m_forceValues.torque.y/100;
        upForce.torque.z += m_forceValues.torque.z/100;
        m_mutex.unlock();
    }
//    pose_world.clear();
//    pose_world.push_back(200);
//    pose_world.push_back(200);
//    pose_world.push_back(200);
//    pose_world.push_back(0);
//    pose_world.push_back(0);
//    pose_world.push_back(M_PI);
//    m_ptrSolver->cartToJnt(currentJoints, pose_world, targetJoint);
    targetJoint.clear();
    targetJoint.push_back(0);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(-M_PI_2);
    targetJoint.push_back(M_PI_2);
    targetJoint.push_back(0);
    m_ptrRobot->move(targetJoint);
    COBOT_LOG.notice()<<"Moved to down position.";
    sleep(3);

    Wrench downForce;
    downForce.force.x = 0;
    downForce.force.y = 0;
    downForce.force.z = 0;
    downForce.torque.x = 0;
    downForce.torque.y = 0;
    downForce.torque.z = 0;
    sleep(1);
    for(int i=0;i<100;i++){
        sleep(0.01);
        m_mutex.lock();
        downForce.force.x += m_forceValues.force.x/100;
        downForce.force.y += m_forceValues.force.y/100;
        downForce.force.z += m_forceValues.force.z/100;
        downForce.torque.x += m_forceValues.torque.x/100;
        downForce.torque.y += m_forceValues.torque.y/100;
        downForce.torque.z += m_forceValues.torque.z/100;
        m_mutex.unlock();
    }
    m_loadGravity.force.x=(upForce.force.x-downForce.force.x)/2.0;
    m_loadGravity.force.y=(upForce.force.y-downForce.force.y)/2.0;
    m_loadGravity.force.z=(upForce.force.z-downForce.force.z)/2.0;

    m_loadGravity.torque.x=(upForce.torque.x-downForce.torque.x)/2.0;
    m_loadGravity.torque.y=(upForce.torque.y-downForce.torque.y)/2.0;
    m_loadGravity.torque.z=(upForce.torque.z-downForce.torque.z)/2.0;


    m_Optoforce_Offset.force.x=(upForce.force.x+downForce.force.x)/2.0;
    m_Optoforce_Offset.force.y=(upForce.force.y+downForce.force.y)/2.0;
    m_Optoforce_Offset.force.z=(upForce.force.z+downForce.force.z)/2.0;
    m_Optoforce_Offset.torque.x=(upForce.torque.x+downForce.torque.x)/2.0;
    m_Optoforce_Offset.torque.y=(upForce.torque.y+downForce.torque.y)/2.0;
    m_Optoforce_Offset.torque.z=(upForce.torque.z+downForce.torque.z)/2.0;
    COBOT_LOG.notice()<<"Calibration done.";
    m_controllerStatus=IDLE;
}
void DragController::DragPhase() {
    //2ms执行一次
    //chrono
    static long time=0;

    static real_T gravity_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    static real_T poseOffset_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    static real_T force_ee_t[6]={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    Eigen::VectorXd currentJoints(6);

    timestamp timenow=std::chrono::high_resolution_clock::now();
    timestamp timeout = timenow + std::chrono::milliseconds(2);
    m_mutex.lock();
    for (int i = 0; i < 6; i++) {
        currentJoints(i) = m_jointValues[i];
        force_ee_t[i]=force_ee[i];
    }
    m_mutex.unlock();
    //计算gravity_ee

    //TODO 修改接口为cv向量表示。

    m_ptrSolver->vector_WorldToEE(currentJoints,
                                  Eigen::Vector3d(m_loadGravity.force.x, m_loadGravity.force.y, m_loadGravity.force.z),
                                  gravity_t_ee);
    gravity_ee[0] = gravity_t_ee.x();
    gravity_ee[1] = gravity_t_ee.y();
    gravity_ee[2] = gravity_t_ee.z();
    gravity_ee[3] = m_loadGravity.torque.x;
    gravity_ee[4] = m_loadGravity.torque.y;
    gravity_ee[5] = m_loadGravity.torque.z;

    m_ptrForceController->step(force_ee_t, gravity_ee, poseOffset_ee);
    if(time%4==0){
        std::vector<double> pose_ee;
        for (int i = 0; i < 6; i++) {
            pose_ee.push_back(poseOffset_ee[i]);
        }
        std::vector<double> pose_world;
        m_ptrSolver->pose_EEToWorld(currentJoints, pose_ee, pose_world);
        std::vector<double> targetJoint;
        //timestamp t1=std::chrono::high_resolution_clock::now();
        m_ptrSolver->cartToJnt(currentJoints, pose_world, targetJoint);
        //timestamp t2=std::chrono::high_resolution_clock::now();
        //std::chrono::duration<double> dur=t2-t1;
        //COBOT_LOG.notice()<<"cartToJnt Execute time:"<<dur.count();
        auto ioStatus = m_ptrRobot->getDigitIoDriver(1);
        if (ioStatus->getIoStatus(DigitIoPort::Port_Ur_Tool_In_0) != DigitIoStatus::Set) {
            m_ptrRobot->move(targetJoint);
        }
    }
    //将控制周期严格限制为2ms。
//        time+=1;
//        if(time%1000==0){
//            COBOT_LOG.notice()<<"time:"<<time;
//        }
    std::this_thread::sleep_until(timeout);
}