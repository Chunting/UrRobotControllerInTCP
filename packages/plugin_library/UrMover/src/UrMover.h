//
// Created by 潘绪洋 on 17-4-26.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_URMOVER_H
#define COBOTSYS_URMOVER_H

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <mutex>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <thread>
#include <deque>
#include <chrono>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <extra2.h>
#include <cxx/cxx.h>
#include <cobotsys_abstract_widget.h>
#include <cobotsys_global_object_factory.h>
#include <QFileDialog>
#include <QPushButton>
#include <cobotsys_file_finder.h>
#include "chain.hpp"
#include "chainiksolverpos_lma.hpp"
#include "chainfksolverpos_recursive.hpp"
#include <Eigen/Dense>
#include <queue>

using namespace cobotsys;
using namespace std;


#define uchar unsigned char
#define JointAngle vector<double>//把关节空间的运动定义为一个特定的名称
#define  ErrorLimit 0.1//用于控制误差范数的一个值，到达这个值标志位清零
#define TCPMAXSPEED 0.8//端最大速度
#define TCPMAXACC 3.5//TCP末端最大加速度

enum MoveMethod{MOVEJ=0,MOVEL=1};
enum Location{START=0,MIDDLE=1,STOP=2};
enum IFThread{MOVET=0,MOVES=1};
enum PlanMethod{Joint=0,Decare=1};
enum StageFlag{Line = 0,Round = 1};
typedef struct threadParameter//全局变量结构体，需要的全局变量都在这个结构体里面
{
    double SendBuf[6];

    double VelBegin[6]={0,0,0,0,0,0};//起始速度
    double VelEnd[6]={0,0,0,0,0,0};//终速度
    //double VelEnd[6]={0.5,0.5,0.5,0.5,0,0};//终速度
    double VelEndOld[6]={0,0,0,0,0,0};//存储一下信息
    std::vector<double> a0;
    std::vector<double> a1;
    std::vector<double> a2;
    std::vector<double> a3;
    std::deque<int> SetPoint;
    int TimeAdd=0;
    int TestCount = 0;
    bool FinishFlag=true;//定义一个标志位，用于判断程序是否执行完毕
    StageFlag MyStageFlag = Line;
    int NumFlag=0;//用于判断执行到第几个动作
    double MyTime=6.5;
    int StepCount=0;
    bool SetFlag=true;//判断是否有新的东西加入了
    double SpeedLimit[6]= {1.6,1.6,1.6,1.6,1.6,1.6};///速度限制，最重要的参数之一，调节这个会改变关节空间规划的运动衫情况
    double ASpeedLimit[6]= {30,30,30,30,30,30};
//    double SpeedLimit[6]= {80,80,80,80,80,80};///速度限制，最重要的参数之一，调节这个会改变关节空间规划的运动衫情况
//    double ASpeedLimit[6]= {15,15,15,5,5,5};
    double TimeMax = 305;//定义一个最长的时间，然后使用二分法获得最小的工作时间
    double PJacobi[6][6] =  //雅克比矩阵的逆矩阵，暂时还用不到不知道以后会不会用到
            {
                    {1.2101,-0.4698,0.0000,0.0106,0.0273,0.0618},
                    {0.9135,1.6976,-0.2617,0.0647,-0.3368,-0.0025},
                    {-1.2560,-2.4801,2.1420,-0.1635,0.7806,-0.0468},
                    {1.0864,0.4936,-1.8803,-0.3576,1.1436,-0.5275},
                    {-1.0934,0.4245,-0.0000,0.1455,0.3747,0.8477},
                    {0.9068,-0.3521,0.0000,0.5799,1.4936,-0.7031},
            };
    std::shared_ptr<AbstractKinematicSolver> m_kinematicSolver;//定义一个用于求逆解的对象
    std::vector<double> AimOld;
    uchar MainOnce = 1;
    uchar MainOnce1 = 1;
    uchar P2pflag = 0;
    double LineP2pCount = 0;
    std::vector<JointAngle> route;
    vector<double> JointNow;
    std::mutex mtx;//搞一个互斥锁来同步数据
    vector<double> targetNow;
    uchar calflag =0;
    uchar Threadflag = 1;
    int ChooseFlagg = 0;
    int LocationFlag = -1;
    int CountFlag = -1;
    int CountFlagOld = -1;
    vector<JointAngle> Route;
    int RatFlag =0;
    int SizeFlag=0;
    int WhichPoint = 0;
    int m_Size = 0;
    bool IFSTART = false;
    vector<double> PointEnd;
    std::deque<JointAngle> SJointAngle;
    std::deque<JointAngle> SDecar;

} ThreadParameter;



class UrMover : public AbstractArmRobotMoveDriver, public ArmRobotRealTimeStatusObserver {
public:
    struct MoveTarget {
        uint32_t moveId;
        cv::Point3d pos;
        cv::Vec3d rpy;
    };

    UrMover();
    virtual ~UrMover();

    virtual bool setup(const QString& configFilePath);
    virtual bool move(uint32_t moveId, const cv::Point3d& pos, const cv::Vec3d& rpy);
    virtual bool move(const std::vector<RobotWaypoint>& waypoints);
    virtual void attach(const std::shared_ptr<ArmRobotMoveStatusObserver>& observer);
    virtual void setRealTimeDriver(const std::shared_ptr<AbstractArmRobotRealTimeDriver>& realTimeDriver);
    virtual void setKinematicSolver(const std::shared_ptr<AbstractKinematicSolver>& kinematicSolver);

    virtual std::shared_ptr<AbstractKinematicSolver>& getKinematicSolver() { return m_kinematicSolver; }

    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

    virtual void clearAttachedObject();

    void moveProcess();

    virtual bool start();
    virtual void clearAll();

    virtual void applyFilter(std::vector<double>& target_, vector<double> & m_target,MoveMethod Method);
    virtual double  DivisionTime(vector<double>& aim,vector<double>& now);//使用二分法获得最短的规划时间
    virtual void  GetDisire(); //新建一个线程用来完成数据的差值运算
    virtual void TrajectoryPlaner3(std::vector<double>& aim, std::vector<double>& now);  //三次多项式规划函数，点对点规划，
    virtual double GetError(vector<double>& target,vector<double>& now);//六参数差值范数
    virtual void PlanerTest(std::vector<double>& target_, vector<double> & m_target);//关节空间规划
    virtual double LineError(vector<double>&a,vector<double>&b);//xyz坐标差值范数
    virtual vector<double> RoundP2p(vector<JointAngle> route,vector<double>& JointNow); //定义一个函数用来表示对一系列进行运动
    virtual void LineP2pGo();//中间函数
    virtual void LineP2pTest(std::vector<double>& target_, vector<double> & m_target);//直线抛物线规划
    virtual void SetPar(Location LocationFlag,int CountFlag,std::vector<std::vector<double>>Route);
    void SetEndVel(vector<double>&A,vector<double>&B,vector<double>&C);
    void GetNextPoint(vector<JointAngle>& Route,vector<double> PointNow);
    bool GetSerialsJoint(std::deque<MoveTarget>& m_targets,vector<double>& Now,PlanMethod Method);
    void STrajectoryPlaner3(std::vector<double>& aim, std::vector<double>& now);
    bool LittleFilter(vector<double>& Now);
    double GetDis7(vector<double> A,vector<double> B);
protected:

    int MyCountFlag = 0;
    std::vector<std::shared_ptr<ArmRobotMoveStatusObserver> > m_observers;
    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_realTimeDriver;
    std::shared_ptr<AbstractKinematicSolver> m_kinematicSolver;
    vector<JointAngle> MyRoute;
//    Eigen::Matrix4d m_cam2base;
//    std::vector<double> m_initialJoint;
    std::vector<double> m_curJoint;
    std::vector<double> m_qcurJoint;
    std::recursive_mutex m_mutex;
    bool m_robotConnected;
    uint64_t m_curJointNum;

    std::deque<MoveTarget> m_targets;
    bool m_clearMoveTarget;

    bool pickMoveTarget(MoveTarget& moveTarget,IFThread A);

    void notify(const MoveTarget& moveTarget, MoveResult moveResult);

    static std::vector<double> toVector(const MoveTarget& moveTarget);
    static double poseDiff(const std::vector<double>& a, const std::vector<double>& b);

    std::thread m_moverThread;
    bool m_exitLoop;
    ThreadParameter MyThreadParameter;//声明一个结构体的全局变量，这样不论是主线程还是其他都可以使用这个变量
};


#endif //COBOTSYS_URMOVER_H
