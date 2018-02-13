//
// Created by 潘绪洋 on 17-5-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "Ur10JointFilter.h"

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;
using namespace cobotsys;
using namespace std;
using namespace KDL;
using namespace std;
Ur10JointFilter::Ur10JointFilter()
{
}

Ur10JointFilter::~Ur10JointFilter()
{
}

bool Ur10JointFilter::setup(const QString& configFilePath)
{
    QJsonObject configJson;
    QString objConfig = FileFinder::find("CONFIG/UrRobotConfig/ur10_180_config.json").c_str();
    if (objConfig.isEmpty())
    {
        COBOT_LOG.notice() << "robot config is empty, robot create fail.";
    }
    else
    {
        COBOT_LOG.notice() << "Succeed loading the config file.";
    }
    auto obj = GlobalObjectFactory::instance()->createObject("KinematicSolverFactory, Ver 1.0", "KinematicSolver");
    if (loadJson(configJson, configFilePath))
    {
        MyThreadParameter.m_kinematicSolver = std::dynamic_pointer_cast<AbstractKinematicSolver>(obj);
        if (MyThreadParameter.m_kinematicSolver)
        {
            if (MyThreadParameter.m_kinematicSolver->setup(objConfig))
            {
                COBOT_LOG.notice() << "Create Setup Solver Success";
            }
            else
            {
                MyThreadParameter.m_kinematicSolver.reset();
            }
        }
        m_jointLimits = readRealArray(configJson["JointLimits"]);
        m_jointAccLimits = readRealArray(configJson["JointAccLimits"]);
        //for (auto& val : m_jointLimits) val = val * M_PI / 180 / 1000 * 8;
       // for (auto& val : m_jointLimits) val = 0.2;
        m_jointLimits[0] = 0.4;
        m_jointLimits[1] = 0.4;
        m_jointLimits[2] = 0.4;
        m_jointLimits[3] = 0.4;
        m_jointLimits[4] = 0.4;
        m_jointLimits[5] = 0.4;

        COBOT_LOG.notice() << "Joint Filter Limits: " << putfixedfloats(6, 1, m_jointLimits, 180 / M_PI);
    }
    return true;
}

void  Ur10JointFilter::GetDisire() //新建一个线程用来完成数据的差值运算
{
    while(MyThreadParameter.Threadflag==1)  //弄个死循环让它一直跑去算了
    {
        double t = 0;
        while (MyThreadParameter.TimeAdd < MyThreadParameter.MyTime * 1000)
        {
            //cout << "2" << endl;
            if(MyThreadParameter.mtx.try_lock())
            {
                t = (double) (MyThreadParameter.TimeAdd) / 1000;
                vector<double>::iterator iter1 = MyThreadParameter.a0.begin();
                vector<double>::iterator iter2 = MyThreadParameter.a1.begin();
                vector<double>::iterator iter3 = MyThreadParameter.a2.begin();
                vector<double>::iterator iter4 = MyThreadParameter.a3.begin();
                for (int i = 0; i < 6; i++)
                {
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
        if(MyThreadParameter.NumFlag<65534)
            MyThreadParameter.NumFlag++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));//必须延时，不然程序跑死了
    }
}
void Ur10JointFilter::TrajectoryPlaner3(std::vector<double>& aim, std::vector<double>& now)  //三次多项式规划函数，点对点规划，
{
    // 控制量为初始及末端位置和初始及末端速度
    std::vector<double> AimNow(aim);
    static uchar Creatflag=0;
    if (MyThreadParameter.AimOld != AimNow)  //判断本次的目标值是否与上次的目标值相同，不要重复执行
    {
        MyThreadParameter.FinishFlag = false;//程序刚刚开始执行，将结束标志位改变为错误
        MyThreadParameter.a0.clear();//忘记清空了。。。。。
        MyThreadParameter.a1.clear();
        MyThreadParameter.a2.clear();
        MyThreadParameter.a3.clear();
        MyThreadParameter.MyTime=DivisionTime(aim,now);
        cout<<"Result Time: "<<MyThreadParameter.MyTime<<endl;
        for (int i = 0; i < aim.size(); i++)  //得到每个关节的三次多项式差值参数
        {
            double a = now[i];
            double b = MyThreadParameter.VelBegin[i];
            double c = (-3 * now[i] + 3 * aim[i] - 2 * MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                        - MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i]) / pow((double) MyThreadParameter.MyTime, (double) 2);
            double d = (2 * now[i] - 2 * aim[i] + MyThreadParameter.MyTime * MyThreadParameter.VelBegin[i]
                        + MyThreadParameter.MyTime * MyThreadParameter.VelEnd[i] )/ pow((double) MyThreadParameter.MyTime, (double) 3);
            MyThreadParameter.a0.push_back(a);
            MyThreadParameter.a1.push_back(b);
            MyThreadParameter.a2.push_back(c);
            MyThreadParameter.a3.push_back(d);
        }
        if(!Creatflag)  //线程只创建一次，不要重复
        {
            std::thread Thread1(&Ur10JointFilter::GetDisire,this);
            Thread1.detach();
            Creatflag = 1;
        }
        MyThreadParameter.TimeAdd = 0;
    }
    MyThreadParameter.AimOld = AimNow;
}


double  Ur10JointFilter::DivisionTime(vector<double>& aim,vector<double>& now) //使用二分法取得最优的规划时间
{
    double Time = MyThreadParameter.TimeMax;
    double TimeLower=0;//二分法的下限
    double TimeHighter = MyThreadParameter.TimeMax;//二分法的上限
    double MiddleTime[6];//二次函数到达峰值的时间
    bool NextTurnFlag = true;//判断下一次迭代是是要向下减少时间，还是向上增加时间，true为减少时间，false为增加时间
    double A0[6],A1[6],A2[6],A3[6];//找好四个关节
    double SpeedMaxStart[6],SpeedMaxStop[6],SpeedMaxMiddle[6];
    double ASpeedMaxStart[6],ASpeedMaxStop[6];

    while((TimeHighter-TimeLower)>0.1)  //二分法的终止条件是我的精度小于0.1S了
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
            else
            {
                SpeedMaxMiddle[i] = SpeedMaxStop[i] =
                        A1[i] + 2 * A2[i] * MiddleTime[i] + 3 * A3[i] * pow(MiddleTime[i], 2.0);
            }
            ASpeedMaxStart[i] = 2 * A2[i];
            ASpeedMaxStop[i] = 2 * A2[i] + 6 * A3[i];
        }
        for (int i = 0; i < aim.size(); i++)  //都是弧度制的，分析是否有超过限制。
        {
            if (fabs(SpeedMaxStart[i]) > MyThreadParameter.SpeedLimit[i])
            {
                NextTurnFlag = false;
                break;
            }
            if (fabs(SpeedMaxStop[i]) > MyThreadParameter.SpeedLimit[i])
            {
                NextTurnFlag = false;
                break;
            }
            if (fabs(SpeedMaxMiddle[i]) > MyThreadParameter.SpeedLimit[i])
            {
                NextTurnFlag = false;
                break;
            }
            if (fabs(ASpeedMaxStart[i]) > MyThreadParameter.ASpeedLimit[i])
            {
                NextTurnFlag = false;
                break;
            }
            if (fabs(ASpeedMaxStop[i]) > MyThreadParameter.ASpeedLimit[i])
            {
                NextTurnFlag = false;
                break;
            }
            NextTurnFlag = true;//如果都满足条件了，当然要改变一下啊
        }
        if(NextTurnFlag)//如果说都能满足要求,则证明时间还有余量，则把上限设置为当前时间
            TimeHighter=Time;
        else//如果说不能满足要求，则证明时间已经太小了，把下限设置为当前时间
            TimeLower=Time;
        Time = (TimeHighter+TimeLower)/2;
        //cout<<TimeHighter<<endl;
    }
    return TimeHighter;
}

double Ur10JointFilter::GetError(vector<double>& target,vector<double>& now)
{
    double Error=0;
    for(int i=0; i<target.size(); i++)
        Error += (target[i]-now[i])*(target[i]-now[i]);//获得误差的范数
    return Error;
}


void Ur10JointFilter::PlanerTest(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus)
{
    //控制两点运动,怎么知道程序是在什么时候改变的？
    {
        vector<double> TestBuf;
        static vector<double> Temp(target_.size(), 0);
        static uchar justone = 1;
        static vector<double> vectorold(6,0);
        static uchar JudgeFlag = 0;
        if ((target_.size() != 0) && (ptrRobotStatus->q_actual.size() != 0))  //只有当向量不为0时，才能够对程序进行执行
        {
            if(justone)
            {
                Temp = ptrRobotStatus->q_actual;
                justone = 0;
                vectorold = ptrRobotStatus->q_actual;
                cout<<"lueluelue"<<endl;
            }
            if ((GetError(target_, ptrRobotStatus->q_actual) > ErrorLimit) &&
                (MyThreadParameter.FinishFlag))//只有在Setflag被置位之后才会执行代入指令
            {
                Temp = target_;
                //COBOT_LOG.notice() << putfixedfloats(7,2,Temp,180/M_PI)<<"  Setting";
                //cout << "Setting  " <<MyThreadParameter.FinishFlag<< endl;
            }
            // COBOT_LOG.notice() << putfixedfloats(7,2,Temp,1)<<" 2 "<<(int)MyThreadParameter.FinishFlag;
            TrajectoryPlaner3(Temp, ptrRobotStatus->q_actual);
            if(MyThreadParameter.mtx.try_lock())
            {
                for (int i = 0; i < target_.size(); i++)
                {
                    target_[i] = MyThreadParameter.SendBuf[i];
                }
                vectorold = target_;
                MyThreadParameter.mtx.unlock();
                //COBOT_LOG.notice() << putfixedfloats(7,2,target_,1)<<" 2 "<<(int)MyThreadParameter.FinishFlag;
            }
            else
                target_=vectorold;
        }
    }
}

void Ur10JointFilter::FilterPlaner(std::vector<double> &target_, const ArmRobotStatusPtr &ptrRobotStatus)
{
    {
        m_target = target_;
        auto curPosi = ptrRobotStatus->q_actual;
        auto curVelo = ptrRobotStatus->qd_actual;

        if (m_target.size() != curPosi.size())
            return;

        if (cxx::distance(curVelo) > 0.001)
        {
//        COBOT_LOG.debug() << "J-Speed: " << putfixedfloats(6, 3, curVelo, 180 / M_PI);
        }

        double maxRadPerTick = 360 * M_PI / 180 / 1000 * 8;
        if (m_jointLimits.size() != m_target.size())
        {
            m_jointLimits.resize(m_target.size(), maxRadPerTick);
            COBOT_LOG.warning() << "Config Joint Limit use Default: " << maxRadPerTick;
        }

        auto posiInc = curVelo;
        std::vector<double> diffNew(m_target.size(), 0);

        cxx::norm_all(posiInc);
        cxx::add_other(posiInc, m_jointAccLimits);

        for (size_t i = 0; i < m_target.size(); i++)
        {
            diffNew[i] = m_target[i] - curPosi[i];

            cxx::limit(diffNew[i], posiInc[i]);

            cxx::limit(diffNew[i], m_jointLimits[i]);

            target_[i] = curPosi[i] + diffNew[i];
        }
    }
}

///***********************TCP梯形差值××××××××××××××××××××××
//考虑加个判断调价 不然逆解总出问题
double Ur10JointFilter::LineError(vector<double>&a,vector<double>&b)
{
    double error = 0;
    for(int i=0; i<3; i++)
    {
        error += (a[i]-b[i])*(a[i]-b[i]);
    }
    return error;
}

vector<double> Ur10JointFilter::RoundP2p(vector<JointAngle> route,vector<double>& JointNow) //定义一个函数用来表示对一系列进行运动
{
    //MyThreadParameter.LineP2pCount = 8000;
    int Size = route.size();
    vector<double> NOW=route[0],NEXT=route[1];
    double Tb = TCPMAXSPEED/TCPMAXACC;
    double Lb = TCPMAXACC * pow(Tb,2.0) *0.5;
    //cout<<Lb<<endl;
    double L= sqrt(pow((NOW[0] -NEXT[1]),2.0)+pow((NOW[2] -NEXT[2]),2.0)+pow((NOW[3] -NEXT[3]),2.0));//求取距离
    double T = 2*Tb + (L-2*Lb)/TCPMAXSPEED;
    double Lblamu = Lb/L;
    double Tblamu = Tb/T;
    double Alafalamu = (2*Lblamu)/pow(Tblamu,2.0);
    double lamu =0 ;
    double t=MyThreadParameter.LineP2pCount / T/1000;
    vector<double> NextPos(JointNow.size(),0);
    double PowNum;
    if(t<=Tblamu)
    {
        PowNum= t*t;
        lamu = 0.5 * Alafalamu * PowNum;
    }
    else if(t<= (1-Tblamu))
        lamu = 0.5 *Alafalamu * pow(Tblamu,2.0) +Alafalamu*Tblamu*(t-Tblamu);
    else if(t <= 1)
        lamu = 0.5 *Alafalamu * pow(Tblamu,2.0) +Alafalamu*Tblamu*(t-Tblamu)-0.5*Alafalamu*pow((t+Tblamu-1),2.0);

    for(int i=0; i<6; i++)
        NextPos[i] = NOW[i] + lamu*(NEXT[i]-NOW[i]);
    vector<double> PosAim=NextPos;
    vector<double> PosReturn(JointNow.size(),0);
    MyThreadParameter.m_kinematicSolver->cartToJnt(JointNow,PosAim,PosReturn);

    if(GetError(NextPos,route[1])<0.00001)
    {
        MyThreadParameter.LineP2pCount=T*1000;
        MyThreadParameter.MainOnce=1;
        MyThreadParameter.MainOnce1 =1;
        MyThreadParameter.P2pflag =0;
        MyThreadParameter.FinishFlag = true;
        COBOT_LOG.notice("Succeeed Point");

    }
    return PosReturn;
}
void Ur10JointFilter::LineP2pGo()
{
    while(1)
    {
        if(!(MyThreadParameter.FinishFlag))
        {
            if(MyThreadParameter.mtx.try_lock())
            {
                // MyThreadParameter.mtx.lock();
                MyThreadParameter.targetNow.clear();
                for (int i = 0; i < 6; i++)
                    MyThreadParameter.targetNow.push_back(0);
                MyThreadParameter.targetNow = RoundP2p(MyThreadParameter.route, MyThreadParameter.JointNow);
                //COBOT_LOG.notice()<<putfixedfloats(7,2,MyThreadParameter.targetNow,M_PI/180);
                MyThreadParameter.calflag  = 1;
                MyThreadParameter.mtx.unlock();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
    }
}
void Ur10JointFilter::LineP2pTest(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus)
{
    static std::vector<double> AimNow(target_.size(),0);
    static std::vector<double> Temp(6,0);
    static vector<double>  AimOld(6,0);
    static vector<double> carnow(6,0);
    static vector<double> jointnow(6,0);
    vector<double> test(6,M_PI);
    static uchar NowOnce =1;
    static uchar outonce =1 ;
    static vector<double> outAim;
    static vector<double> outAim1;
    static uchar firstflag =1;//是否是第一次被执行？
    static uchar Creatflag=0;
    static vector<double> JointOld(6,0);
    jointnow = ptrRobotStatus->q_actual;
    if(MyThreadParameter.mtx.try_lock())
    {
        if ((target_.size() != 0) && (jointnow.size() != 0))
        {
            if (MyThreadParameter.FinishFlag)
            {
                AimNow = target_;
                // cout<<"New Aim"<<endl;
            }
            double Derror = GetError(AimNow, jointnow);
            if (MyThreadParameter.FinishFlag && Derror > 0.1)
            {
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
                    MyThreadParameter.JointNow.push_back(ptrRobotStatus->q_actual[i]);
                if (!Creatflag)  //线程只创建一次，不要重复
                {
                    std::thread Thread2(&Ur10JointFilter::LineP2pGo,this);
                    Thread2.detach();
                    Creatflag = 1;
                }
            }
            if (!MyThreadParameter.FinishFlag)
            {
                //MyThreadParameter.mtx.lock();//数据同步
                target_ = MyThreadParameter.targetNow;
                JointOld = target_;
                //COBOT_LOG.notice()<<putfixedfloats(7,2,target_,1);//抖动是因为根本没有进入这个东西
                MyThreadParameter.JointNow.clear();
                for (int i = 0; i < 6; i++)
                    MyThreadParameter.JointNow.push_back(ptrRobotStatus->q_actual[i]);
                MyThreadParameter.LineP2pCount += 8;
                //COBOT_LOG.notice() << putfixedfloats(7,2,target_,1);
                MyThreadParameter.mtx.unlock();
            }
        }
        MyThreadParameter.mtx.unlock();
    }
    else
        target_ = JointOld;
    //就是得另起一个线程，不然主程序会卡死
}

void Ur10JointFilter::SetEndVel(vector<double>&A,vector<double>&B,vector<double>&C){
    if(A.size()&&B.size()&&C.size()){
        for(int i =0;i<6;i++){
            if((C[i]-B[i])*(B[i]-A[i])<=0)
                this->MyThreadParameter.VelEnd[i] = 0;
            else{
                this->MyThreadParameter.VelEnd[i]=((B[i]-A[i])>0)?(this->MyThreadParameter.SpeedLimit[i]/2)
                                                  :(-1*this->MyThreadParameter.SpeedLimit[i]/2);
            }
        }
    }
}


void Ur10JointFilter::SetPar(int LocationFlag,int CountFlag,std::vector<std::vector<double>>Route) {
    //cout<<"lueluelue"<<endl;
    this->MyThreadParameter.CountFlag = CountFlag;
    if (CountFlag != this->MyThreadParameter.CountFlagOld) {
        if (Route.size() < 3) {
            for (int i = 0; i < 6; i++) {
                this->MyThreadParameter.VelBegin[i] = 0;
                this->MyThreadParameter.VelEnd[i] = 0;
            }
        } else {
            switch (LocationFlag) {
                case 0: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = 0;
                    SetEndVel(Route[0], Route[1], Route[2]);
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEndOld[i] = this->MyThreadParameter.VelEnd[i];
                }
                case 1: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = this->MyThreadParameter.VelEndOld[i];
                    SetEndVel(Route[0], Route[1], Route[2]);
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEndOld[i] = this->MyThreadParameter.VelEnd[i];
                }
                case 2: {
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelBegin[i] = this->MyThreadParameter.VelEndOld[i];
                    for (int i = 0; i < 6; i++)
                        this->MyThreadParameter.VelEnd[i] = 0;
                }
            }
        }
        this->MyThreadParameter.CountFlagOld= CountFlag;
    }
}
bool Ur10JointFilter::GetFinishFlag() {
    return this->MyThreadParameter.FinishFlag;
}
void Ur10JointFilter::LittleFilter(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus){
    if((target_.size()>0)&&(ptrRobotStatus->q_actual.size()>0)) {
        m_target = target_;
        auto curPosi = ptrRobotStatus->q_actual;

        double maxRadPerTick = 60 * M_PI / 180 / 1000 * 8;
        if (m_jointLimits.size() != m_target.size()) {
            m_jointLimits.resize(m_target.size(), maxRadPerTick);
            COBOT_LOG.warning() << "Config Joint Limit use Default: " << maxRadPerTick;
        }

        std::vector<double> diffNew(m_target.size(), 0);
        for (size_t i = 0; i < m_target.size(); i++) {
            diffNew[i] = m_target[i] - curPosi[i];

            if (diffNew[i] > m_jointLimits[i])//是否又超出限制
                diffNew[i] = m_jointLimits[i];
            if (diffNew[i] < -m_jointLimits[i])
                diffNew[i] = -m_jointLimits[i];
            target_[i] = curPosi[i] + diffNew[i];
        }
    }
}

void Ur10JointFilter::applyFilter(std::vector<double>& target_, const ArmRobotStatusPtr& ptrRobotStatus) {
    //FilterPlaner(target_,ptrRobotStatus);//简单滤波
    //PlanerTest(target_,ptrRobotStatus);//关节角三次多项式规划运动
    //LineP2pTest(target_,ptrRobotStatus);//TCP直线运动
   // LittleFilter(target_,ptrRobotStatus);
//    //COBOT_LOG.debug() << putfixedfloats(7, 2, diffNew, 180 / M_PI);
}

