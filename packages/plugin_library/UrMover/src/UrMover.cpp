//
// Created by 潘绪洋 on 17-4-26.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "UrMover.h"
#include <Eigen/Dense>
#include <extra2.h>
using namespace Eigen;

UrMover::UrMover() {
    QJsonObject configJson;
    QString objConfig = FileFinder::find("CONFIG/UrRobotConfig/ur10_180_config.json").c_str();
    if (objConfig.isEmpty()) {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
    } else {
        COBOT_LOG.notice() << "Succeed loading the config file.";
    }
    auto obj = GlobalObjectFactory::instance()->createObject("KinematicSolverFactory, Ver 1.0", "KinematicSolver");
    MyThreadParameter.m_kinematicSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
    if (MyThreadParameter.m_kinematicSolver) {
        if (MyThreadParameter.m_kinematicSolver->setup(objConfig)) {
            COBOT_LOG.notice() << "Create Setup Solver Success";
        } else {
            MyThreadParameter.m_kinematicSolver.reset();
        }
    }
    m_robotConnected = true;
    m_curJointNum = 0;
    m_exitLoop = false;
}
UrMover::~UrMover() {
    m_exitLoop = true;
    if (m_moverThread.joinable()) {
        m_moverThread.join();
    }
}
bool UrMover::setup(const QString &configFilePath) {

    return true;
}

bool UrMover::move(uint32_t moveId, const cv::Point3d &pos, const cv::Vec3d &rpy) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_robotConnected) {
        cv::Point3d POS1;
        cv::Vec3d rpy1;
        if(MyThreadParameter.TestCount==0) {
            //m_targets.push_back({moveId, pos, rpy});
             POS1 = {-0.72928, 0.20981, 0.42024};
             rpy1 = {-M_PI, 0, -M_PI / 2};
            m_targets.push_back({moveId, POS1, rpy1});
            POS1 = {-0.38508, 0.77732, 0.37024};
            rpy1 = {-M_PI, 0, -M_PI / 2};
            m_targets.push_back({moveId, POS1, rpy1});
            POS1 = {-0.38508, 0.77732, 0.07024};
            rpy1 = {-M_PI, 0, -M_PI / 2};
            m_targets.push_back({moveId, POS1, rpy1});
            MyThreadParameter.TestCount=1;
        }
        else {
            POS1 = {-0.38508, 0.77732, 0.37024};
            rpy1 = {-M_PI, 0, -M_PI / 2};
            m_targets.push_back({moveId, POS1, rpy1});
            POS1 = {-0.72928, 0.20981, 0.42024};
            m_targets.push_back({moveId, POS1, rpy1});
            POS1 = {-0.43963, 0.29184, -0.26388};
            rpy1 = {-M_PI, 0, -M_PI / 2};
            m_targets.push_back({moveId, POS1, rpy1});
            MyThreadParameter.TestCount=0;
        }
        MyThreadParameter.SizeFlag = m_targets.size();
        COBOT_LOG.debug() << "Total Targets : " << m_targets.size();
        return true;
    } else {
    }
    return false;
}

bool UrMover::move(const std::vector<RobotWaypoint> &waypoints) {
    // TODO 计算路径，规划完整的Joint
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    cout << "oooooooooooooooooooooooo" << endl;
    if (m_robotConnected) {
        for (int i = 0; i < waypoints.size(); i++) {
            m_targets.push_back({waypoints[i].moveId, waypoints[i].position, waypoints[i].rpy});
            COBOT_LOG.debug() << "Total Targets : " << m_targets.size();
        }
        MyThreadParameter.SizeFlag = m_targets.size();
        return true;
    }
    return false;
}

void UrMover::attach(const std::shared_ptr<ArmRobotMoveStatusObserver> &observer) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    for (auto &iter : m_observers) {
        if (iter == observer) {
            return;
        }
    }
    m_observers.push_back(observer);
}

void UrMover::setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver> &realTimeDriver) {
    m_realTimeDriver = realTimeDriver;
    m_realTimeDriver->attach(std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(shared_from_this()));
}

void UrMover::setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver> &kinematicSolver) {
    m_kinematicSolver = kinematicSolver;
}


void UrMover::onArmRobotConnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = true;
}

void UrMover::onArmRobotDisconnect() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_robotConnected = false;
    clearAll();
    m_curJointNum++;
}

void UrMover::onArmRobotStatusUpdate(const ArmRobotStatusPtr &ptrRobotStatus) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    m_curJoint = ptrRobotStatus->q_actual;
    m_qcurJoint = ptrRobotStatus->qd_actual;
    m_curJointNum++;
}

void UrMover::notify(const UrMover::MoveTarget &moveTarget, MoveResult moveResult) {
    std::vector<std::shared_ptr<ArmRobotMoveStatusObserver> > observers;
    m_mutex.lock();
    observers = m_observers;
    m_mutex.unlock();

    for (auto &ob : observers) {
        if (ob) {
            ob->onMoveFinish(moveTarget.moveId, moveResult);
        }
    }
}


void UrMover::moveProcess() {
    auto timePoint = std::chrono::high_resolution_clock::now();
    std::vector<double> joint;
    std::vector<double> curPose;
    std::vector<double> targetJoint;
    uint64_t jointNum = 0;
    uint64_t jointNumOld = 0;
    MoveTarget moveTarget;
    bool moveFinished = true;
    bool noMoveTarget = true;
    int delay_flag = 0;//加一个小延时防止发生累计误差
    vector<double> EndPoint(6, 0);
    bool clearAction = false;
    static int CountAngle = 0;
    double actual_pose_diff;
    double pose_err_last = 0;
    static int VVV=0;

    auto hres_start = std::chrono::high_resolution_clock::now();
    while (!m_exitLoop) {
        timePoint = timePoint + std::chrono::milliseconds(1);
        m_mutex.lock();
        jointNum = m_curJointNum;
        joint = m_curJoint;
        clearAction = m_clearMoveTarget;
        m_clearMoveTarget = false;
        m_mutex.unlock();

        m_kinematicSolver->jntToCart(joint, curPose);

        if (clearAction) {
            clearAction = false;
            if (!noMoveTarget) {
                noMoveTarget = true;
                moveFinished = true;
                COBOT_LOG.notice() << "Mover Target has canceled, " << jointNum;
                hres_start = std::chrono::high_resolution_clock::now();
                notify(moveTarget, MoveResult::Cancled);
            }
        }

        if (jointNum > jointNumOld) {
            // Pick target
            if (noMoveTarget) {
                if (pickMoveTarget(moveTarget, MOVES)) {
                    hres_start = std::chrono::high_resolution_clock::now();
                    moveFinished = false;
                    noMoveTarget = false;
                    MyCountFlag = MyCountFlag % MyThreadParameter.SizeFlag;
                    MyCountFlag++;
                    COBOT_LOG.notice() << std::setw(5) << moveTarget.moveId
                                       << " Mover Target: " << moveTarget.pos << ", " << moveTarget.rpy;
                }
            }

            // Go Target
            if (!noMoveTarget)  //如果说存在移动目标的话
            {
                auto pose = toVector(moveTarget);
                actual_pose_diff = poseDiff(pose, curPose); // 目标与实际的位置误差

                auto error_prev = fabs(actual_pose_diff - pose_err_last);

//                COBOT_LOG.debug() << "Mover Error: " << actual_pose_diff << ", " << error_prev;
                //cout<<moveTarget.moveId<<" "<<endl;
                double ErrorRidus = 0;
                if (m_targets.size() > 0) {
                    ErrorRidus = 0;
                } else {
                    ErrorRidus = 0.005;
                }
                if (actual_pose_diff < ErrorRidus ||
                    (actual_pose_diff < 1 && error_prev < 0.001)) {
                    if (!moveFinished)  //这还是干什么？什么鬼？
                    {
                        auto hres_cur = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> time_diff = hres_cur - hres_start;
                        moveFinished = true;//这个是防止目标点一直被抓取的，一次运动我的目标点只抓取一次
                        //noMoveTarget = true;//达到这个条件后，我才认为这个运动完成了

                        notify(moveTarget, MoveResult::Success);
                        COBOT_LOG.notice() << std::setw(5) << moveTarget.moveId
                                           << " Mover Target: " << moveTarget.pos << ", " << moveTarget.rpy
                                           << " Finished. Time: " << time_diff.count() * 1000 << "ms";
                    }
                } else {
                    if (m_kinematicSolver->cartToJnt(joint, pose, targetJoint) == 0) {
                        //这里得看好Joint是目标角度
                        // TODO smooth target joint commands
                        //这里是接口，MOVEJ是关节空间运动，MOVEL是直线运动
                        /*if (MyCountFlag == 1) {
                            GetNextPoint(MyRoute, targetJoint);
                            SetPar(START, MyCountFlag, MyRoute);
                        } else if (m_targets.size() == 0) {
                            GetNextPoint(MyRoute, targetJoint);
                            SetPar(STOP, MyCountFlag, MyRoute);
                        } else {
                            GetNextPoint(MyRoute, targetJoint);
                            SetPar(MIDDLE, MyCountFlag, MyRoute);
                        }
                        applyFilter(targetJoint,m_curJoint,MOVEJ);
                        m_realTimeDriver->move(targetJoint);*/
                        //if (GetSerialsJoint(m_targets, m_curJoint,Joint)) {//Decare
                        //COBOT_LOG.notice() << "Aim Now  " << putfixedfloats(7, 2, m_curJoint, 180/M_PI);
                        if (GetSerialsJoint(m_targets, m_curJoint,Decare)) {
                            cout << "luelueluelueluelueluelueluelueluelueluelueluelueluelueluelue" << endl;
                            //moveTarget.moveId = 1;
                            CountAngle = 0;
                        }
                        if (MyThreadParameter.SJointAngle.size() > 0) {
                            MyThreadParameter.IFSTART = true;
                            targetJoint = MyThreadParameter.SJointAngle.front();
                            //targetJoint[5]=m_curJoint[5];
                            MyThreadParameter.SJointAngle.pop_front();
                           // LittleFilter(targetJoint);
                            m_realTimeDriver->move(targetJoint);//移动输出
                            //COBOT_LOG.notice() <<"Speed before"<< putfixedfloats(7, 2, m_qcurJoint, 1);
                            CountAngle++;
                            if (CountAngle > MyThreadParameter.SetPoint.front()) {
                                //if(MyThreadParameter.WhichPoint < MyThreadParameter.m_Size)
                                notify(moveTarget, MoveResult::Success);
                                moveTarget.moveId++;
                                MyThreadParameter.SetPoint.pop_front();
                                MyThreadParameter.WhichPoint++;
                                actual_pose_diff = poseDiff(MyThreadParameter.PointEnd, curPose);
                                cout <<"------------------------------------"<<moveTarget.moveId<< "   Current Difference  " << actual_pose_diff << endl;
                            }

                        }
                    } else {//达不到才会清除目标
                        COBOT_LOG.error() << "Target: " << moveTarget.pos << ", " << moveTarget.rpy
                                          << " Can Not Reached!";
                        notify(moveTarget, MoveResult::InvalidMoveTarget);
                        clearAll();
                    }
                }
                pose_err_last = actual_pose_diff;
            }//截止到这里是目标存在
        }
        jointNumOld = jointNum;
        std::this_thread::sleep_until(timePoint);

    }
    COBOT_LOG.notice() << "UrMover is stopped.";
}

#define FilASpeedLimit 0.1
bool UrMover::LittleFilter(vector<double>& Now){
    vector<double> SpeedNow(6,0);
    vector<double> SpeedNext(6,0);
    vector<double> NextPoint(6,0);
    vector<double> ASpeed(6,0);
    SpeedNow = m_qcurJoint;
    NextPoint =MyThreadParameter.SJointAngle.front();
    for(int i=0;i<6;i++){
        SpeedNext[i] = (Now[i]-m_curJoint[i])*125;
    }
    for(int i=0;i<6;i++){
        ASpeed[i] = (SpeedNext[i]-SpeedNow[i])*125;
    }
//    for(int i=0;i<6;i++){
//        if(ASpeed[i]>MyThreadParameter.SpeedLimit[i]){
//
//        }
//    }
   // COBOT_LOG.notice() <<"ASpeed NOW"<< putfixedfloats(7, 2, SpeedNow, 1);
}


void UrMover::clearAttachedObject() {
    m_mutex.lock();
    m_exitLoop = true;
    m_mutex.unlock();

    if (m_moverThread.joinable()) {
        m_moverThread.join();
    }

    m_mutex.lock();
    m_observers.clear();
    m_kinematicSolver.reset();
    m_realTimeDriver.reset();
    m_mutex.unlock();
}

bool UrMover::pickMoveTarget(UrMover::MoveTarget &moveTarget, IFThread A) {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_targets.size()) {
        moveTarget = m_targets.front();
        if (A == MOVET)
            m_targets.pop_front();//取了一次数值之后上面的就被抛出了
        return true;
    }
    return false;
}

std::vector<double> UrMover::toVector(const UrMover::MoveTarget &moveTarget) {
    std::vector<double> j;
    j.push_back(moveTarget.pos.x);
    j.push_back(moveTarget.pos.y);
    j.push_back(moveTarget.pos.z);
    j.push_back(moveTarget.rpy[0]);
    j.push_back(moveTarget.rpy[1]);
    j.push_back(moveTarget.rpy[2]);
    return j;
}

double UrMover::poseDiff(const std::vector<double> &a, const std::vector<double> &b) {
    double diff_sum = 0;
    if (a.size() == b.size()) {
        std::vector<double> diff(a.size(), 0);
        for (size_t i = 0; i < a.size(); i++) {
            diff[i] = a[i] - b[i];
            if (i >= 3) {
                diff[i] = diff[i] * 180 / M_PI;
                if (abs(diff[i]) > 300)//RPY角度是存在问题的
                    diff[i] = 0;
            } else {
                diff[i] = diff[i] * 1000;
            }
            diff[i] = diff[i] * diff[i];

            diff_sum += diff[i];
        }
    }
    //cout<<sqrt(diff_sum)<<endl;
    return sqrt(diff_sum);//范数开根号？
}

bool UrMover::start() {
    std::lock_guard<std::recursive_mutex> lockGuard(m_mutex);
    if (m_kinematicSolver && m_realTimeDriver) {
        m_moverThread = std::thread(&UrMover::moveProcess, this);
        return true;
    }
    return false;
}

void UrMover::clearAll() {
    std::deque<MoveTarget> tmpTargets;

    COBOT_LOG.notice() << "UrMover: Clear ALL";

    m_mutex.lock();
    tmpTargets = m_targets;
    m_targets.clear();
    m_clearMoveTarget = true;
    auto num_tgt = tmpTargets.size();
    m_mutex.unlock();

    for (auto &iter : tmpTargets) {
        notify(iter, MoveResult::Cancled);
    }
}


/**
 * xxxxxxxxxxxxxxxxxxxxxxxxxxxx轨迹规划用的xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
 */

void JointALimit(vector<double> &A) {//用来让关节角不要超过幅度的
    for (int i = 0; i < 6; i++) {
        if (A[i] > 2 * M_PI)
            A[i] = A[i] - 2 * M_PI;
        else if (A[i] < -2 * M_PI)
            A[i] += 2 * M_PI;
    }
}

void UrMover::GetNextPoint(vector<JointAngle > &Route, vector<double> PointNow) {
    if (m_targets.size() > 1) {//有两个或两个以上的点
        Route.clear();
        Route.push_back(PointNow);
        vector<double> A = toVector(m_targets[0]);
        vector<double> B = toVector(m_targets[1]);
        vector<double> A1(6, 0);
        vector<double> B1(6, 0);
        m_kinematicSolver->cartToJnt(PointNow, A, A1);
        JointALimit(A1);
        Route.push_back(A1);
        m_kinematicSolver->cartToJnt(A1, B, B1);
        JointALimit(B1);
        Route.push_back(B1);
    } else {
        Route.clear();//如果再没有两个点的话，那我就直接给他输入现在的他的三个点就OK
        Route.push_back(PointNow);
        Route.push_back(PointNow);
        //Route.push_back(PointNow);
    }
}

void UrMover::GetDisire() //新建一个线程用来完成数据的差值运算
{
    while (MyThreadParameter.Threadflag == 1)  //弄个死循环让它一直跑去算了
    {
        double t = 0;
        while (MyThreadParameter.TimeAdd < MyThreadParameter.MyTime * 1000) {
            //cout << "2" << endl;
            if (MyThreadParameter.mtx.try_lock()) {
                t = (double) (MyThreadParameter.TimeAdd) / 1000;
                vector<double>::iterator iter1 = MyThreadParameter.a0.begin();
                vector<double>::iterator iter2 = MyThreadParameter.a1.begin();
                vector<double>::iterator iter3 = MyThreadParameter.a2.begin();
                vector<double>::iterator iter4 = MyThreadParameter.a3.begin();
                for (int i = 0; i < 6; i++) {
                    MyThreadParameter.SendBuf[i] =
                            *iter1 + *iter2 * t + *iter3 * pow(t, (double) 2) + *iter4 * pow(t, (double) 3);
                    iter1++;
                    iter2++;
                    iter3++;
                    iter4++;
                }
                //cout<<MyThreadParameter.TimeAdd<<endl;
                MyThreadParameter.mtx.unlock();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(8));
            MyThreadParameter.TimeAdd += 8;//表示的是毫秒级的数值
        }
        MyThreadParameter.FinishFlag = true;//程序执行完毕，将结束标志位置真
        if (MyThreadParameter.NumFlag < 65534)
            MyThreadParameter.NumFlag++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));//必须延时，不然程序跑死了
    }
}

void UrMover::TrajectoryPlaner3(std::vector<double> &aim, std::vector<double> &now)  //三次多项式规划函数，点对点规划，
{
    // 控制量为初始及末端位置和初始及末端速度
    std::vector<double> AimNow(aim);
    static uchar Creatflag = 0;
    if (MyThreadParameter.AimOld != AimNow)  //判断本次的目标值是否与上次的目标值相同，不要重复执行
    {
        MyThreadParameter.FinishFlag = false;//程序刚刚开始执行，将结束标志位改变为错误
        MyThreadParameter.a0.clear();//忘记清空了。。。。。
        MyThreadParameter.a1.clear();
        MyThreadParameter.a2.clear();
        MyThreadParameter.a3.clear();
        MyThreadParameter.MyTime = DivisionTime(aim, now);
        cout << "Result Time: " << MyThreadParameter.MyTime << endl;
        for (int i = 0; i < aim.size(); i++)  //得到每个关节的三次多项式差值参数
        {
            double a = now[i];
            double b = MyThreadParameter.VelBegin[i];
            double c = (-3 * now[i] + 3 * aim[i] - 2 * MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                        - MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i]) /
                       pow((double) MyThreadParameter.MyTime, (double) 2);
            double d = (2 * now[i] - 2 * aim[i] + MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                        + MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i]) /
                       pow((double) MyThreadParameter.MyTime, (double) 3);
            MyThreadParameter.a0.push_back(a);
            MyThreadParameter.a1.push_back(b);
            MyThreadParameter.a2.push_back(c);
            MyThreadParameter.a3.push_back(d);
        }
        if (!Creatflag)  //线程只创建一次，不要重复
        {
            std::thread Thread1(&UrMover::GetDisire, this);
            Thread1.detach();
            Creatflag = 1;
        }
        MyThreadParameter.TimeAdd = 0;
    }
    MyThreadParameter.AimOld = AimNow;
}


double UrMover::DivisionTime(vector<double> &aim, vector<double> &now) //使用二分法取得最优的规划时间
{
    double Time = MyThreadParameter.TimeMax;
    double TimeLower = 0;//二分法的下限
    double TimeHighter = MyThreadParameter.TimeMax;//二分法的上限
    double MiddleTime[6];//二次函数到达峰值的时间
    bool NextTurnFlag = true;//判断下一次迭代是是要向下减少时间，还是向上增加时间，true为减少时间，false为增加时间
    double A0[6], A1[6], A2[6], A3[6];//找好四个关节
    double SpeedMaxStart[6], SpeedMaxStop[6], SpeedMaxMiddle[6];
    double ASpeedMaxStart[6], ASpeedMaxStop[6];

    while ((TimeHighter - TimeLower) > 0.005)  //二分法的终止条件是我的精度小于0.1S了
    {
        for (int i = 0; i < aim.size(); i++)  //得到每个关节的三次多项式差值参数
        {
            double a = now[i];
            double b = MyThreadParameter.VelBegin[i];
            double c = (-3 * now[i] + 3 * aim[i] - 2 * Time * MyThreadParameter.VelBegin[i]
                        - Time * MyThreadParameter.VelEnd[i]) / pow((double) Time, (double) 2);
            double d = (2 * now[i] - 2 * aim[i] + Time * MyThreadParameter.VelBegin[i]
                        + Time * MyThreadParameter.VelEnd[i]) / pow((double) Time, (double) 3);
            A0[i] = a;
            A1[i] = b;
            A2[i] = c;
            A3[i] = d;
        }
        for (int i = 0; i < aim.size(); i++)  //获得每个关节的加速度和速度的值，用来判断时间是大了还是小了
        {
            MiddleTime[i] = (-1) * A2[i] / A3[i] / 3;
            SpeedMaxStart[i] = A1[i];
            SpeedMaxStop[i] = A1[i] + 2 * A2[i] * Time + 3 * A3[i] * pow(Time, 2.0);
            if ((MiddleTime[i] < 0) || (MiddleTime[i] > Time))
                SpeedMaxMiddle[i] = 0;
            else {
                SpeedMaxMiddle[i] =
                        A1[i] + 2 * A2[i] * MiddleTime[i] + 3 * A3[i] * pow(MiddleTime[i], 2.0);
            }
            ASpeedMaxStart[i] = 2 * A2[i];
            ASpeedMaxStop[i] = 2 * A2[i] + 6 * A3[i];
        }
        for (int i = 0; i < aim.size(); i++)  //都是弧度制的，分析是否有超过限制。
        {
            if (fabs(SpeedMaxStart[i]) > MyThreadParameter.SpeedLimit[i]) {
                NextTurnFlag = false;
                break;
            }
            if (fabs(SpeedMaxStop[i]) > MyThreadParameter.SpeedLimit[i]) {
                NextTurnFlag = false;
                break;
            }
            if (fabs(SpeedMaxMiddle[i]) > MyThreadParameter.SpeedLimit[i]) {
                NextTurnFlag = false;
                break;
            }
            if (fabs(ASpeedMaxStart[i]) > MyThreadParameter.ASpeedLimit[i]) {
                NextTurnFlag = false;
                break;
            }
            if (fabs(ASpeedMaxStop[i]) > MyThreadParameter.ASpeedLimit[i]) {
                NextTurnFlag = false;
                break;
            }
            NextTurnFlag = true;//如果都满足条件了，当然要改变一下啊
        }
        if (NextTurnFlag)//如果说都能满足要求,则证明时间还有余量，则把上限设置为当前时间
            TimeHighter = Time;
        else//如果说不能满足要求，则证明时间已经太小了，把下限设置为当前时间
            TimeLower = Time;
        Time = (TimeHighter + TimeLower) / 2;
    }
    cout<<"+++++++++++++++++++++++++++++++++";
        for(int i =0;i<6;i++){
            cout<<SpeedMaxMiddle[i]<<" ";
        }
    cout<<endl;
    cout<<"+++++++++++++++++++++++++++++++++";
    for(int i =0;i<6;i++){
        cout<<MyThreadParameter.SpeedLimit[i]<<" ";
    }
    cout<<endl;

    return TimeHighter;
}

double UrMover::GetError(vector<double> &target, vector<double> &now) {
    double Error = 0;
    for (int i = 0; i < target.size(); i++)
        Error += (target[i] - now[i]) * (target[i] - now[i]);//获得误差的范数
    return Error;
}


void UrMover::PlanerTest(std::vector<double> &target_, vector<double> &m_target) {
    //控制两点运动,怎么知道程序是在什么时候改变的？
    {
        vector<double> TestBuf;
        static vector<double> Temp(target_.size(), 0);
        static uchar justone = 1;
        static vector<double> vectorold(6, 0);
        static uchar JudgeFlag = 0;
        if ((target_.size() != 0) && (m_target.size() != 0))  //只有当向量不为0时，才能够对程序进行执行
        {
            if (justone) {
                Temp = m_target;
                justone = 0;
                vectorold = m_target;
                cout << "lueluelue" << endl;
            }
            if ((GetError(target_, m_target) > ErrorLimit) &&
                (MyThreadParameter.FinishFlag))//只有在Setflag被置位之后才会执行代入指令
            {
                Temp = target_;
                //COBOT_LOG.notice() << putfixedfloats(7,2,Temp,180/M_PI)<<"  Setting";
                //cout << "Setting  " <<MyThreadParameter.FinishFlag<< endl;
            }
            // COBOT_LOG.notice() << putfixedfloats(7,2,Temp,1)<<" 2 "<<(int)MyThreadParameter.FinishFlag;
            TrajectoryPlaner3(Temp, m_target);
            if (MyThreadParameter.mtx.try_lock()) {
                for (int i = 0; i < target_.size(); i++) {
                    target_[i] = MyThreadParameter.SendBuf[i];
                }
                vectorold = target_;
                MyThreadParameter.mtx.unlock();
            } else
                target_ = vectorold;
        }
    }
}


///***********************TCP梯形差值××××××××××××××××××××××
//考虑加个判断调价 不然逆解总出问题
double UrMover::LineError(vector<double> &a, vector<double> &b) {
    double error = 0;
    for (int i = 0; i < 3; i++) {
        error += (a[i] - b[i]) * (a[i] - b[i]);
    }
    return error;
}

vector<double> UrMover::RoundP2p(vector<JointAngle > route, vector<double> &JointNow) //定义一个函数用来表示对一系列进行运动
{
    //MyThreadParameter.LineP2pCount = 8000;
    int Size = route.size();
    vector<double> NOW = route[0], NEXT = route[1];
    double Tb = TCPMAXSPEED / TCPMAXACC;
    double Lb = TCPMAXACC * pow(Tb, 2.0) * 0.5;
    //cout<<Lb<<endl;
    double L = sqrt(pow((NOW[0] - NEXT[0]), 2.0) + pow((NOW[1] - NEXT[1]), 2.0) + pow((NOW[2] - NEXT[2]), 2.0));//求取距离
    double T = 2 * Tb + (L - 2 * Lb) / TCPMAXSPEED;
    double Lblamu = Lb / L;
    double Tblamu = Tb / T;
    double Alafalamu = (2 * Lblamu) / pow(Tblamu, 2.0);
    double lamu = 0;
    double t = MyThreadParameter.LineP2pCount / T / 1000;
    vector<double> NextPos(JointNow.size(), 0);
    double PowNum;
    if (t <= Tblamu) {
        PowNum = t * t;
        lamu = 0.5 * Alafalamu * PowNum;
    } else if (t <= (1 - Tblamu))
        lamu = 0.5 * Alafalamu * pow(Tblamu, 2.0) + Alafalamu * Tblamu * (t - Tblamu);
    else if (t <= 1)
        lamu = 0.5 * Alafalamu * pow(Tblamu, 2.0) + Alafalamu * Tblamu * (t - Tblamu) -
               0.5 * Alafalamu * pow((t + Tblamu - 1), 2.0);

    for (int i = 0; i < 3; i++)
        NextPos[i] = NOW[i] + lamu * (NEXT[i] - NOW[i]);
    for (int i =3;i<6;i++)
        NextPos[i] = NOW[i];
    vector<double> PosAim = NextPos;
   // COBOT_LOG.notice()<<putfixedfloats(7,3,PosAim,1);
    vector<double> PosReturn(JointNow.size(), 0);
    //MyThreadParameter.SDecar.push_back(PosAim);
    m_kinematicSolver->cartToJnt(JointNow, PosAim, PosReturn);
    if(MyThreadParameter.StepCount==0) {
        if (t > (1 - Tblamu))//这时就要进行圆弧规划了
            MyThreadParameter.MyStageFlag = Round;
    }
    if (GetError(NextPos, route[1]) < 0.00001) {
        MyThreadParameter.LineP2pCount = T * 1000;
        MyThreadParameter.MainOnce = 1;
        MyThreadParameter.MainOnce1 = 1;
        MyThreadParameter.P2pflag = 0;
        MyThreadParameter.FinishFlag = true;
        MyThreadParameter.MyStageFlag = Round;
        COBOT_LOG.notice("Succeeed Point");

    }
    return PosReturn;
}

void UrMover::LineP2pGo() {
    while (1) {
        if (!(MyThreadParameter.FinishFlag)) {
            if (MyThreadParameter.mtx.try_lock()) {
                // MyThreadParameter.mtx.lock();
                MyThreadParameter.targetNow.clear();
                for (int i = 0; i < 6; i++)
                    MyThreadParameter.targetNow.push_back(0);
                MyThreadParameter.targetNow = RoundP2p(MyThreadParameter.route, MyThreadParameter.JointNow);
                //COBOT_LOG.notice()<<putfixedfloats(7,2,MyThreadParameter.targetNow,M_PI/180);
                MyThreadParameter.calflag = 1;
                MyThreadParameter.mtx.unlock();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}

void UrMover::LineP2pTest(std::vector<double> &target_, vector<double> &m_target) {
    static std::vector<double> AimNow(target_.size(), 0);
    static std::vector<double> Temp(6, 0);
    static vector<double> AimOld(6, 0);
    static vector<double> carnow(6, 0);
    static vector<double> jointnow(6, 0);
    vector<double> test(6, M_PI);
    static uchar NowOnce = 1;
    static uchar outonce = 1;
    static vector<double> outAim;
    static vector<double> outAim1;
    static uchar firstflag = 1;//是否是第一次被执行？
    static uchar Creatflag = 0;
    static vector<double> JointOld(6, 0);
    jointnow = m_target;
    if (MyThreadParameter.mtx.try_lock()) {
        if ((target_.size() != 0) && (jointnow.size() != 0)) {
            if (MyThreadParameter.FinishFlag) {
                AimNow = target_;
                // cout<<"New Aim"<<endl;
            }
            double Derror = GetError(AimNow, jointnow);
            if (MyThreadParameter.FinishFlag && Derror > 0.1) {
                MyThreadParameter.m_kinematicSolver->jntToCart(AimNow, Temp);
                MyThreadParameter.m_kinematicSolver->jntToCart(jointnow, AimOld);
                MyThreadParameter.route.clear();
                MyThreadParameter.route.push_back(AimOld);
                MyThreadParameter.route.push_back(Temp);
                MyThreadParameter.LineP2pCount = 0;
                MyThreadParameter.FinishFlag = 0;
                MyThreadParameter.JointNow.clear();
                MyThreadParameter.Threadflag = 0;
                // cout<<"lueluelue"<<endl;
                for (int i = 0; i < 6; i++)
                    MyThreadParameter.JointNow.push_back(m_target[i]);
                if (!Creatflag)  //线程只创建一次，不要重复
                {
                    std::thread Thread2(&UrMover::LineP2pGo, this);
                    Thread2.detach();
                    Creatflag = 1;
                }
            }
            if (!MyThreadParameter.FinishFlag) {
                //MyThreadParameter.mtx.lock();//数据同步
                target_ = MyThreadParameter.targetNow;
                JointOld = target_;
                //COBOT_LOG.notice()<<putfixedfloats(7,2,target_,1);//抖动是因为根本没有进入这个东西
                MyThreadParameter.JointNow.clear();
                for (int i = 0; i < 6; i++)
                    MyThreadParameter.JointNow.push_back(m_target[i]);
                MyThreadParameter.LineP2pCount += 8;
                //COBOT_LOG.notice() << putfixedfloats(7,2,target_,1);
                MyThreadParameter.mtx.unlock();
            }
        }
        MyThreadParameter.mtx.unlock();
    } else
        target_ = JointOld;
    //就是得另起一个线程，不然主程序会卡死
}

#define EndSpeed 0.31
#define EndSpeed1 0

void UrMover::SetEndVel(vector<double> &A, vector<double> &B, vector<double> &C) {
    if (A.size() && B.size() && C.size()) {
        for (int i = 0; i < 6; i++) {
            if ((C[i] - B[i]) * (B[i] - A[i]) < 0) {
                this->MyThreadParameter.VelEnd[i] = ((C[i] - B[i]) > 0) ? EndSpeed1 : -EndSpeed1;
            } else if ((C[i] - B[i]) * (B[i] - A[i]) > 0) {
                this->MyThreadParameter.VelEnd[i] = ((B[i] - A[i]) > 0) ? EndSpeed
                                                                        : -EndSpeed;
            } else {
                if ((C[i] - B[i]) == 0)
                    this->MyThreadParameter.VelEnd[i] = ((B[i] - A[i]) > 0) ? 0
                                                                            : -0;
                else
                    this->MyThreadParameter.VelEnd[i] = ((C[i] - B[i]) > 0) ? EndSpeed
                                                                            : -EndSpeed;
            }
        }
    }

}


void UrMover::SetPar(Location LocationFlag, int CountFlag, std::vector<std::vector<double>> Route) {
    //cout<<"lueluelue"<<endl;
    this->MyThreadParameter.CountFlag = CountFlag;
    if (CountFlag != this->MyThreadParameter.CountFlagOld) {
        if (Route.size() < 3) {
            for (int i = 0; i < 6; i++) {
                this->MyThreadParameter.VelBegin[i] = 0;
                this->MyThreadParameter.VelEnd[i] = 0;
            }
            cout << MyCountFlag << " stop " << MyThreadParameter.VelBegin[0] << " "
                 << MyThreadParameter.VelBegin[1] << " " << MyThreadParameter.VelBegin[2] << " "
                 << MyThreadParameter.VelEnd[0] << " " << MyThreadParameter.VelEnd[1] << " "
                 << MyThreadParameter.VelEnd[2] << endl;
        } else {
            switch (LocationFlag) {
                case START: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = 0;
                    SetEndVel(Route[0], Route[1], Route[2]);
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEndOld[i] = this->MyThreadParameter.VelEnd[i];
                    cout << MyCountFlag << " start " << MyThreadParameter.VelBegin[0] << " "
                         << MyThreadParameter.VelBegin[1] << " " << MyThreadParameter.VelBegin[2] << " "
                         << MyThreadParameter.VelEnd[0] << " " << MyThreadParameter.VelEnd[1] << " "
                         << MyThreadParameter.VelEnd[2] << endl;

                    break;
                }
                case MIDDLE: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = this->MyThreadParameter.VelEndOld[i];
                    SetEndVel(Route[0], Route[1], Route[2]);
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEndOld[i] = this->MyThreadParameter.VelEnd[i];
                    cout << MyCountFlag << " middle " << MyThreadParameter.VelBegin[0] << " "
                         << MyThreadParameter.VelBegin[1] << " " << MyThreadParameter.VelBegin[2] << " "
                         << MyThreadParameter.VelEnd[0] << " " << MyThreadParameter.VelEnd[1] << " "
                         << MyThreadParameter.VelEnd[2] << endl;

                    break;
                }
                case STOP: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = this->MyThreadParameter.VelEndOld[i];
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEnd[i] = 0;
                    cout << MyCountFlag << " stop " << MyThreadParameter.VelBegin[0] << " "
                         << MyThreadParameter.VelBegin[1] << " " << MyThreadParameter.VelBegin[2] << " "
                         << MyThreadParameter.VelEnd[0] << " " << MyThreadParameter.VelEnd[1] << " "
                         << MyThreadParameter.VelEnd[2] << endl;

                    break;
                }
            }
        }
        this->MyThreadParameter.CountFlagOld = CountFlag;

    }
}

void UrMover::applyFilter(std::vector<double> &target_, vector<double> &m_target, MoveMethod Method) {
    if (Method == MOVEJ)
        PlanerTest(target_, m_target);//关节角三次多项式规划运动
    else
        LineP2pTest(target_, m_target);//TCP直线运动
}

//换一种思路来试试,把所有数据都压进一个容器里面去
void UrMover::STrajectoryPlaner3(std::vector<double> &aim, std::vector<double> &now)  //三次多项式规划函数，点对点规划，
{
    // 控制量为初始及末端位置和初始及末端速度
    //COBOT_LOG.notice() <<endl<<endl << putfixedfloats(7, 2, now, 1);
    MyThreadParameter.a0.clear();//忘记清空了。。。。。
    MyThreadParameter.a1.clear();
    MyThreadParameter.a2.clear();
    MyThreadParameter.a3.clear();
    MyThreadParameter.MyTime = DivisionTime(aim, now);
    cout << "Result Time: " << MyThreadParameter.MyTime << endl;
    for (int i = 0; i < aim.size(); i++)  //得到每个关节的三次多项式差值参数
    {
        double a = now[i];
        double b = MyThreadParameter.VelBegin[i];
        double c = (-3 * now[i] + 3 * aim[i] - 2 * MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                    - MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i]) /
                   pow((double) MyThreadParameter.MyTime, (double) 2);
        double d = (2 * now[i] - 2 * aim[i] + MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                    + MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i]) /
                   pow((double) MyThreadParameter.MyTime, (double) 3);
        MyThreadParameter.a0.push_back(a);
        MyThreadParameter.a1.push_back(b);
        MyThreadParameter.a2.push_back(c);
        MyThreadParameter.a3.push_back(d);
    }
}
//把所有的数据都压到一个队列里面去
bool UrMover::GetSerialsJoint(std::deque<MoveTarget> &m_targets, vector<double> &Now,PlanMethod Method) {
    //if(MyThreadParameter.mtx.try_lock()) {
    MyCountFlag = 0;
    vector<double> Aim(6, 0);
    vector<double> AimJ(6, 0);
    vector<double> AimFirst(6, 0);
    vector<double> NowCopy(6, 0);
    vector<double> NowDcr(6,0);//当前位置的笛卡尔坐标
    MoveTarget target;
    double Tb1 = TCPMAXSPEED / TCPMAXACC;
    double Lb1 = TCPMAXACC * pow(Tb1, 2.0) * 0.5;//获得位置速度信息
    int JCount = 0;
    if(Method == Joint) { //使用关节空间进行规划
        int i = 0;
        double t = 0;
        double CountTime = 0;
        int Size = m_targets.size();
        if (m_targets.size() > 0) {
            MyThreadParameter.m_Size = m_targets.size();
            MyThreadParameter.TimeMax = 305;
            MyThreadParameter.SetPoint.clear();
            //cout<<"The Size is:"<<m_targets.size()<<endl;
            MyThreadParameter.SJointAngle.clear();
            for (i = 0; i < Size; i++) {
                MyThreadParameter.TimeMax = 305;
                if (i >= 5) {
                    for (int p = 0; p < 6; p++) {
                        MyThreadParameter.SpeedLimit[p] = 2.8;
                        MyThreadParameter.ASpeedLimit[p] = 24;
                    }
                } else {
                    if (i == 4 || i == 1) {
                        for (int p = 0; p < 6; p++) {
                            MyThreadParameter.SpeedLimit[p] = 1.7;
                            MyThreadParameter.ASpeedLimit[p] = 14;
                        }
                    } else {
                        for (int p = 0; p < 6; p++) {
                            MyThreadParameter.SpeedLimit[p] = 2.0;
                            MyThreadParameter.ASpeedLimit[p] = 24;
                        }
                    }
                }
                cout << "///////////////////////////////////////";
                for (int p = 0; p < 6; p++) {
                    cout << MyThreadParameter.SpeedLimit[p] << " ";
                }
                cout << endl;
                if (i > 0)
                    Now = AimJ;
                NowCopy = Now;
                // COBOT_LOG.notice() <<endl<<endl << putfixedfloats(7, 2, NowCopy, 1)<<endl<<putfixedfloats(7, 2, AimJ, 1);
                target = m_targets.front();
                m_targets.pop_front();
                MyCountFlag++;
                Aim = toVector(target);
                if (m_targets.size() == 0) {
                    MyThreadParameter.PointEnd.clear();
                    for (int k = 0; k < 6; k++) {
                        MyThreadParameter.PointEnd.push_back(Aim[k]);
                    }
                }
                m_kinematicSolver->cartToJnt(NowCopy, Aim, AimJ);
                if (MyCountFlag == 1) {
                    GetNextPoint(MyRoute, Now);
                    SetPar(START, MyCountFlag, MyRoute);
                } else if (m_targets.size() == 0) {
                    GetNextPoint(MyRoute, Now);
                    SetPar(STOP, MyCountFlag, MyRoute);
                } else {
                    GetNextPoint(MyRoute, Now);
                    SetPar(MIDDLE, MyCountFlag, MyRoute);
                }
                STrajectoryPlaner3(AimJ, NowCopy);
                MyThreadParameter.MyTime *= 1000;
                for (CountTime = 0; CountTime < MyThreadParameter.MyTime; CountTime += 8) {
                    t = (double) (CountTime) / 1000.0;
                    vector<double>::iterator iter1 = MyThreadParameter.a0.begin();
                    vector<double>::iterator iter2 = MyThreadParameter.a1.begin();
                    vector<double>::iterator iter3 = MyThreadParameter.a2.begin();
                    vector<double>::iterator iter4 = MyThreadParameter.a3.begin();
                    for (int j = 0; j < 6; j++) {
                        AimFirst[j] = *iter1 + *iter2 * t + *iter3 * pow(t, (double) 2) + *iter4 * pow(t, (double) 3);
                        iter1++;
                        iter2++;
                        iter3++;
                        iter4++;
                    }
                    MyThreadParameter.SJointAngle.push_back(AimFirst);
                    //COBOT_LOG.notice() << "Aim Now  " << putfixedfloats(7, 2, AimFirst, 1) << "  count " << t;
                    JCount++;
                }
                int Test = JCount - 1;
                MyThreadParameter.SetPoint.push_back(Test);
            }
            //    cout << "The i is :" << i << "  " << "The Size is:" << Size << endl;
            return true;
        } else
            return false;
    }
    else{//使用笛卡尔空间规划
        int Size1 = m_targets.size();//计数变量
        MyThreadParameter.MyStageFlag = Line;
        vector<double> DesireJoint(6,0);
        int count = 0;
        int countpop = 0;
        if(m_targets.size()>0){//只有当其中有数据才会进行
            MyThreadParameter.StepCount = 0;
            MyThreadParameter.SJointAngle.clear();
            for(count=0;count <Size1;count++){
                if(MyThreadParameter.MyStageFlag == Line){//当现在需要进行的直线运动
                    MyThreadParameter.LineP2pCount = 0;//先把时间清零
                    if(count>0) {
                        //Now = AimJ;
                        MyThreadParameter.LineP2pCount = Tb1*1000;//如果不是第一个的话就不能从头开始
                    }
                    vector<double> A5 (6,0);
                    for(int i = 0;i<6;i++)
                        A5[i] = Now[i] *180 / M_PI;
                    MyThreadParameter.route.clear();
                    MyThreadParameter.m_kinematicSolver->jntToCart(Now,NowDcr);
                    if(count==0)
                        MyThreadParameter.route.push_back(NowDcr);
                    else
                        MyThreadParameter.route.push_back(Aim);
                    target = m_targets.front();
                    m_targets.pop_front();
                    countpop++;
                    cout<<"11111111111111111111111111111111   "<<countpop<<endl;
                    Aim = toVector(target);
                    MyThreadParameter.route.push_back(Aim);
                    COBOT_LOG.notice() << "Aim Now  " << putfixedfloats(7, 2, Now, 180/M_PI)<<endl<<
                                                      putfixedfloats(7, 2, MyThreadParameter.route[0], 1)<<endl
                                                                                                            <<putfixedfloats(7, 2, MyThreadParameter.route[1], 1);
                    if(m_targets.size()==0)
                        MyThreadParameter.StepCount = 1;
                    while(MyThreadParameter.MyStageFlag==Line){
                        DesireJoint = RoundP2p(MyThreadParameter.route,Now);
                        MyThreadParameter.SJointAngle.push_back(DesireJoint);
                        MyThreadParameter.LineP2pCount+=8;
                        JCount++;
                    }
                    int Test1 = JCount - 1;
                    MyThreadParameter.SetPoint.push_back(Test1);
                    cout<<"222222222222222222222222222  "<< MyThreadParameter.SJointAngle.size()<<endl;
                }
                if(MyThreadParameter.MyStageFlag == Round) {//开始圆弧规划
                    if (m_targets.size() > 0) {

                            m_kinematicSolver->jntToCart(DesireJoint, NowDcr);//获得第一个点的笛卡尔坐标
                            //target还是代表目标点
                            vector<double> RoundTarget(6, 0);
                            target = m_targets.front();
                            RoundTarget = toVector(target);//圆弧要达到的位置
                            double Errorx = RoundTarget[0] - Aim[0];
                            double Errory = RoundTarget[1] - Aim[1];
                            double Errorz = RoundTarget[2] - Aim[1];
                            double ErrorDis = sqrt(pow(Errorx, 2.0) + pow(Errory, 2.0) + pow(Errorz, 2.0));
                            vector<double> RoundReal(6, 0);
                            RoundReal = RoundTarget;
                            RoundReal[0] = Aim[0] + Lb1 / ErrorDis * Errorx;
                            RoundReal[1] = Aim[1] + Lb1 / ErrorDis * Errory;
                            RoundReal[2] = Aim[2] + Lb1 / ErrorDis * Errorz;//找到圆弧的另外一个点
                            //现在使用三个点确定一个圆弧分别是NowDcr,Aim,RoundReal
                            vector<double> Point1 = NowDcr;
                            vector<double> Point2 = Aim;
                            vector<double> Point3 = RoundReal;
                            //下面使用Eigen库进行计算
                            MatrixXd MA1(3, 3);
                            MatrixXd MB1(3, 3);
                            MatrixXd MC1(3, 3);
                            MatrixXd MD1(3, 3);
                            MA1 << Point1[1], Point1[2], 1,
                                    Point2[1], Point2[2], 1,
                                    Point3[1], Point3[2], 1;
                            MB1 << Point1[0], Point1[2], 1,
                                    Point2[0], Point2[2], 1,
                                    Point3[0], Point3[2], 1;
                            MC1 << Point1[0], Point1[1], 1,
                                    Point2[0], Point2[1], 1,
                                    Point3[0], Point3[1], 1;
                            MD1 << Point1[0], Point1[1], Point1[2],
                                    Point2[0], Point2[1], Point2[2],
                                    Point3[0], Point3[1], Point3[2];
                            double A1 = MA1.determinant();
                            double B1 = -1.0 * MB1.determinant();
                            double C1 = MC1.determinant();
                            double D1 = MD1.determinant();
                            double A2 = Point2[0] - Point1[0];
                            double B2 = Point2[1] - Point1[1];
                            double C2 = Point2[2] - Point1[2];
                            double D2 = 0.5 * (pow(Point1[0], 2.0) + pow(Point1[1], 2.0) + pow(Point1[2], 2.0)
                                               - pow(Point2[0], 2.0) - pow(Point2[1], 2.0) - pow(Point2[2], 2.0));
                            double A3 = Point3[0] - Point1[0];
                            double B3 = Point3[1] - Point1[1];
                            double C3 = Point3[2] - Point1[2];
                            double D3 = 0.5 * (pow(Point1[0], 2.0) + pow(Point1[1], 2.0) + pow(Point1[2], 2.0)
                                               - pow(Point3[0], 2.0) - pow(Point3[1], 2.0) - pow(Point3[2], 2.0));
                            A2 = 2 * A2;
                            B2 = 2 * B2;
                            C2 = 2 * C2;
                            D2 = 2 * D2;

                            A3 = 2 * A3;
                            B3 = 2 * B3;
                            C3 = 2 * C3;
                            D3 = 2 * D3;
                            MatrixXd ZD1(3, 3);
                            ZD1 << A1, B1, C1,
                                    A2, B2, C2,
                                    A3, B3, C3;
                            Vector3d Center;
                            Vector3d DD(D1, D2, D3);
                            Center = ZD1.inverse() * DD;
                            Center = -1 * Center;
                            vector<double> CenterPoint(6, 0);
                            CenterPoint = Point1;
                            CenterPoint[0] = Center(0);
                            CenterPoint[1] = Center(1);
                            CenterPoint[2] = Center(2);
                            vector<double> CenterPointTrue(6, 0);//上面那个圆心是假的只不过是一个同时过三个点的圆心
                            //这个圆心才是真正相切的圆心，利用这个圆去求各个点就ok
                            CenterPointTrue = CenterPoint;
                            for (int i = 0; i < 3; i++)
                                CenterPointTrue[i] = CenterPoint[i] + (CenterPoint[i] - Point2[i]);
                            //现在的任务是找到对应的转换矩阵
                            Vector3d U;//末端点减初始点
                            Vector3d U1;//末端点减初始点
                            Vector3d V;
                            Vector3d W(A1, B1, C1);//UVW系
                            for (int i = 0; i < 3; i++)
                                U(i) = Point1[i] - CenterPointTrue[i];
                            for (int i = 0; i < 3; i++)
                                U1(i) = Point3[i] - CenterPointTrue[i];
                            double R = sqrt(pow(U(0), 2.0) + pow(U(1), 2.0) + pow(U(2), 2.0));
                            double Theta = TCPMAXSPEED / R / 1000 * 8*0.85;//一个执行周期内可以转过的角度
                            U.normalize();
                            W = U.cross(U1);
                            W.normalize();
                            double xxd = U.dot(W);
                            V = W.cross(U);
                            Matrix4d TR;//齐次旋转阵
                            TR << U(0), V(0), W(0), CenterPointTrue[0],
                                    U(1), V(1), W(1), CenterPointTrue[1],
                                    U(2), V(2), W(2), CenterPointTrue[2],
                                    0, 0, 0, 1;
                            Matrix4d InvTR = TR;
                            InvTR = TR.inverse();//对齐次旋转阵进行求逆
                            Vector4d ExtendedP1(Point1[0], Point1[1], Point1[2], 1);
                            Vector4d ExtendedP3(Point3[0], Point3[1], Point3[2], 1);
                            ExtendedP1 = InvTR * ExtendedP1;
                            ExtendedP3 = InvTR * ExtendedP3;//找到两点在平面系里的坐标
                            double AngleP3 = acos(
                                    ExtendedP3(0) / sqrt(pow(ExtendedP3(0), 2.0) + pow(ExtendedP3(1), 2.0)));
                            double NextX = 0, NextY = 0;
                            int ccount = 0;
                            vector<double> NextPos(6, 0);
                            vector<double> NextJot(6, 0);
                            Theta = AngleP3 > 0 ? Theta : (-1 * Theta);
                            for (double ThisAngle = 0; ThisAngle <= AngleP3; ThisAngle += Theta) {
                                if (ccount > 0)
                                    Now = NextJot;
                                NextX = R * cos(ThisAngle);
                                NextY = R * sin(ThisAngle);
                                Vector4d ExtendedNextP(NextX, NextY, 0, 1);
                                ExtendedNextP = TR * ExtendedNextP;
                                NextPos = NowDcr;
                                for (int i = 0; i < 3; i++)
                                    NextPos[i] = ExtendedNextP(i);
                                if (m_kinematicSolver->cartToJnt(Now, NextPos, NextJot) == -4) {
                                    cout << "??????????????" << endl;
                                }
                                MyThreadParameter.SJointAngle.push_back(NextJot);
                                ccount++;
                            }
                            MyThreadParameter.MyStageFlag = Line;
                            cout << "222222222222222222222222222  " << MyThreadParameter.SJointAngle.size() << endl;
                    }
                }
            }
            return true;
        } else
            return false;
    }
}

double UrMover::GetDis7(vector<double> A,vector<double> B){
    double Test;
    double SUM=0;
    for(int i=0;i<3;i++){
        SUM+=pow((A[i]-B[i]),2.0);
    }
    Test = sqrt(SUM);
    return Test;
}




//笛卡尔空间关节轨迹规划


