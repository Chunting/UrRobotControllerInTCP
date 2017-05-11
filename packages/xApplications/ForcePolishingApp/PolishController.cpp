//
// Created by 杨帆 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "PolishController.h"
typedef std::chrono::high_resolution_clock::time_point timestamp;


PolishController::PolishController() :m_stop(false),force_control_en(false){
	m_ptrForceController.reset(new ForceControllerClass());
	m_jointValues.resize(6);
    m_loadGravity.force=cv::Point3d(0.0,0.0,1);
    m_loadGravity.torque=cv::Point3d(0.0,0.0,0.0);
    m_controllerStatus=IDLE;
    for(int i=0;i<6;i++){
        force_ee[i] = 0.0;
    }
}

PolishController::~PolishController() {
    INFO_DESTRUCTOR(this);
}

void PolishController::onStartDrag() {
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

void PolishController::onStopDrag() {
    m_stop=true;
    quit();
    if (m_ptrForceSensor) {
		m_ptrForceSensor->stop();
	}
	if (m_ptrRobot) {
		m_ptrRobot->stop();
	}
}


bool PolishController::setup(const QString& configFilePath) {
	bool result=true;
	result&=createArmRobotDriver(configFilePath);
	result&=createSolver(configFilePath);
	result&=createForceSensor();

    //TODO 暂时使用默认路径，导入ptd文件路径和stl文件路径。
    m_model_path=FileFinder::find("CONFIG/ForcePolishing/phone_shell_bin.stl");
    m_polishingTask.parseSTL(m_model_path);
    m_ptd_path=FileFinder::find("CONFIG/ForcePolishing/PolishingTask_test7.ptd");//
    m_polishingTask.parsePTD(m_ptd_path);

    cartesian_path_generator();///< Generate cartesian path of EE
    defineInterimPath();///< insert interim path

    return result;
}

bool PolishController::createArmRobotDriver(const QString& configFilePath) {
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
			auto pFilter = GlobalObjectFactory::instance()->createObject("Ur10FilterFactory, Ver 1.0", "Ur10JointFilter");
			pFilter->setup("CONFIG/FilterSpeed.json");
			auto ppf = std::dynamic_pointer_cast<ArmRobotJointTargetFilter>(pFilter);

			m_ptrRobot->setTargetJointFilter(ppf);
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


bool PolishController::createSolver(const QString& configFilePath) {
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

bool PolishController::createForceSensor() {
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

void PolishController::onArmRobotConnect() {
	//TODO 这个接口是否可以提供一个初步的实现。然后可以在继承接口时，可以不必实现。
}

void PolishController::onArmRobotDisconnect() {

}

void PolishController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
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

void PolishController::onForceSensorConnect() {

}
void PolishController::onForceSensorDisconnect() {

}
void PolishController::onForceSensorDataStreamUpdate(const std::shared_ptr<Wrench>& ptrWrench) {

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
void PolishController::onMoveFinish(uint32_t moveId) {

}
void PolishController::setControllerStatus(PolishController::Controller_Status status){
    m_controllerStatus=status;
    COBOT_LOG.notice()<<"the drag controller status is set to "<< (int)status;
}
void PolishController::run() {
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
                PolishPhase();
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
void PolishController::GravityCalib(){
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
void PolishController::PolishPhase() {
    //2ms执行一次
    //chrono
    static long time=0;

    static real_T gravity_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    static real_T poseOffset_ee[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    static real_T force_ee_t[6]={ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    static real_T fz_touch=0.0;
    static real_T disOffset=0.0;
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
    //TODO parseTzTouch
    m_ptrSolver->vector_WorldToEE(currentJoints,
                                  Eigen::Vector3d(m_loadGravity.force.x, m_loadGravity.force.y, m_loadGravity.force.z),
                                  gravity_t_ee);
    gravity_ee[0] = gravity_t_ee.x();
    gravity_ee[1] = gravity_t_ee.y();
    gravity_ee[2] = gravity_t_ee.z();
    gravity_ee[3] = m_loadGravity.torque.x;
    gravity_ee[4] = m_loadGravity.torque.y;
    gravity_ee[5] = m_loadGravity.torque.z;
    fz_touch=parseFzTouch();
    m_ptrForceController->step(fz_touch, disOffset);


    if(time%4==0){
        std::vector<double> targetJoint;
        //TODO MotionPlanner
        std::vector<double> actualTarget=MotionPlanner(disOffset);
        //timestamp t1=std::chrono::high_resolution_clock::now();
        m_ptrSolver->cartToJnt(currentJoints, actualTarget, targetJoint);
        //timestamp t2=std::chrono::high_resolution_clock::now();
        //std::chrono::duration<double> dur=t2-t1;
        //COBOT_LOG.notice()<<"cartToJnt Execute time:"<<dur.count();
        m_ptrRobot->move(targetJoint);
    }
    //将控制周期严格限制为2ms。
//        time+=1;
//        if(time%1000==0){
//            COBOT_LOG.notice()<<"time:"<<time;
//        }
    std::this_thread::sleep_until(timeout);
}
double PolishController::parseFzTouch(){
    //1.polishing point在ee下的frame。
    //force_ee*polishingFrame中的Fz分量即为所求。
    return 0.0;
}
std::vector<double>  PolishController::MotionPlanner(double disOffset){
    //norminal frame 1,2
    std::vector<double> actualTarget;
    if(force_control_en){
        //
    }else{
        //cartesian_path_;
        return actualTarget;
    }
}
void PolishController::cartesian_path_generator() {
    COBOT_LOG.info()<<"The version of CPolishingTask is "<<m_polishingTask.VERSION_STR.data();
    //Get T_CPi
    std::vector<std::vector<KDL::Frame> >cart_path_cp=m_polishingTask.getCartesianTrajectory();

    //This information is stored in ptd files, CPolishingTask should parse this information and provide it.
    //We simplified T_AB equals to E currently, assume that the Original point of robot is equal to world origin. It can expend if necessary.
    //Input: Output: cartesian_path_
    //T_BC=(T_AB^-1)*T_AE*(T_CP^-1);
    KDL::Frame Frame_AB=m_polishingTask.getRobotFrame();
    for (int i = 0; i < cart_path_cp.size(); ++i) {
        std::vector<KDL::Frame> cart_segment_cp=cart_path_cp[i];
        std::vector<KDL::Frame> cart_segment_bc;
        //Get T_AE. Currently we simplified contact point only have a frame. It will expend to multiple points if necessary.
        KDL::Frame cart_frame_ae=getPolisherFrame(i);
        for (int j = 0; j < cart_segment_cp.size(); ++j) {
            cart_segment_bc.push_back(Frame_AB.Inverse()*cart_frame_ae*cart_segment_cp[j].Inverse());
        }
        cartesian_path_.push_back(cart_segment_bc);
    }
}
KDL::Frame PolishController::getPolisherFrame(int id) {
    KDL::Frame cart_frame_ae = KDL::Frame();
    //KDL::Frame cart_frame_ae = m_polishingTask.getPolisherFrames().at(id);
    //TODO将旋转角度暂时设置为0.
    std::vector<double> angle_data;
    angle_data.push_back(0.0);
    cart_frame_ae.M = cart_frame_ae.M * KDL::Rotation::RotY(0);
    //cart_frame_ae.M = cart_frame_ae.M * KDL::Rotation::RotY(angle_data.at(id));
    return cart_frame_ae;
}
void PolishController::defineInterimPath() {
    interim_path_.clear();
    long nsize = cartesian_path_.size();
    unsigned long last;
    for (int i = 0; i < nsize; ++i) {
        std::vector<KDL::Frame> frames;
        std::vector<KDL::Frame> intPath;
        last = cartesian_path_[i].size() - 1;
        if (checkPathContinuity(i)) {
            intPath.push_back(cartesian_path_[i][last]);
            COBOT_LOG.notice()<<"path continue" << i;
        } else {
            COBOT_LOG.notice()<<"insert interim path" << i;
            intPath=planInterimPath(i);
        }
        interim_path_.push_back(intPath);
    }
}
bool PolishController::checkPathContinuity(int path_index) {
    long nsize = cartesian_path_.size();
    long last = cartesian_path_[path_index].size() - 1;

    KDL::Frame last_frame = cartesian_path_[path_index][last], next_frame;
    if (path_index == nsize - 1)
        next_frame = cartesian_path_[0][0];
    else
        next_frame = cartesian_path_[path_index + 1][0];
    if (KDL::Equal(last_frame, next_frame, 2e-3))//2mm
    {
        if (path_index == nsize - 1)
            next_frame = cartesian_path_[0][0];
        else
            next_frame = cartesian_path_[path_index + 1][0];
        return true;
    }
    return false;
}
std::vector<KDL::Frame> PolishController::planInterimPath(int path_index) {
    long nsize = cartesian_path_.size();
    long last = cartesian_path_[path_index].size() - 1;

    KDL::Frame last_frame = cartesian_path_[path_index][last], next_frame;
    if (path_index == nsize - 1)
        next_frame = cartesian_path_[0][0];
    else
        next_frame = cartesian_path_[path_index + 1][0];

    last_frame.p.data[2] += 0.06;
    next_frame.p.data[2] += 0.06;
    std::vector<KDL::Frame> frames;
    frames.push_back(last_frame);
    frames.push_back(next_frame);
    return frames;
}