//
// Created by 杨帆 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "DragController.h"
typedef std::chrono::high_resolution_clock::time_point timestamp;


DragController::DragController() :m_stop(false){
	m_ptrForceController.reset(new ForceControllerClass());
	m_jointValues.resize(6);
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
        }
    }
	return false;
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
        }
    }
	return false;
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
	m_mutex.unlock();
    Q_EMIT forceUpdated(m_forceValues);
}
void DragController::onMoveFinish(uint32_t moveId) {

}

void DragController::run() {
	//2ms执行一次
	//chrono
    //static long time=0;
	static real_T force_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	static real_T gravity_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	static real_T poseOffset_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	Eigen::VectorXd currentJoints(6);
    while(!m_stop){
        timestamp timenow=std::chrono::high_resolution_clock::now();
        timestamp timeout = timenow + std::chrono::milliseconds(2);
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
        //将控制周期严格限制为2ms。
//        time+=2;
//        if(time%1000==0){
//            COBOT_LOG.notice()<<"time:"<<time;
//        }
        std::this_thread::sleep_until(timeout);
    }
}
