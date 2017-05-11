//
// Created by mingpeng on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef FORCELOGICIO_H
#define FORCELOGICIO_H

#include <vector>
#include <QObject>
#include <thread>
#include <mutex>
#include <cobotsys_abstract_controller.h>
//#include <../MotomanRobotDriver/src/MotomanRobotDriver.h>

#define READY_IN    0
#define REACH_IN    1

using namespace cobotsys;

enum LogicState{LOGIC1, LOGIC2, LOGIC3, DEFSTATE};
typedef void (*FUNC)(void *);

class ForceLogicIO : public QObject, public AbstractController
{   
Q_OBJECT
public:
    ForceLogicIO();
    ~ForceLogicIO();
    bool    setup(const QString& configFilePath);
    bool    start();
    void    stop();   
    void    pause();
    void    keepon();
    bool    getState();
    bool    setLogic(bool _auto, LogicState _logic = DEFSTATE);
    
protected:
    void    threadLogicIO();
    bool    getIO(int _IO);
    void    setIO(int _IO);
    //template <class T>
    //bool registStatsandFunc(T _t, FUNC _f);
    
    virtual void    logic1Process();
    virtual void    logic2Process();
    virtual void    logic3Process();
    virtual void    defaultProcess();
    
private:
    std::mutex                  m_mutex;
    std::thread                 m_thread;
    bool                        m_isStarted;
    bool                        m_isPaused;
    LogicState                  m_LogicState;
    bool                        m_isBusy;
    bool                        m_isTargetReady;
    bool                        m_isTargetReach;
    bool                        m_isAuto;
    //template <class T>
    //static std::vector<T>       m_LogicState;
    //static std::vector<FUNC>    m_LogicProcess;
};

#endif // FORCELOGICIO_H
