

#include "ForceLogicIO.h"

ForceLogicIO::ForceLogicIO():m_isStarted(false),
    m_isPaused(false),
    m_LogicState(DEFSTATE),
    m_isBusy(false),
    m_isTargetReach(false),
    m_isTargetReady(false),
    m_isAuto(false)
{
    COBOT_LOG.info() << "Already start, if want restart, stop first";
    printf("Process ForceLogicIO Start...\n");
}

ForceLogicIO::~ForceLogicIO()
{
    m_thread.join();
}

bool ForceLogicIO::setup(const QString& configFilePath) {
    return true;
}

bool ForceLogicIO::start()
{
    if(m_isStarted) return false;
    printf("Process ForceLogicIO Start...\n");
    setLogic(true);
    m_thread = std::thread(&ForceLogicIO::threadLogicIO, this);
    return true;
}

void ForceLogicIO::stop()
{
    m_isStarted = false;
}

bool ForceLogicIO::setLogic(bool _auto, LogicState _logic)
{
    m_isAuto = _auto;
    if(!_auto) m_LogicState = _logic;
    else return false;
    return true;
}

void ForceLogicIO::threadLogicIO()
{
    m_isStarted = true;
    while(m_isStarted)
    {
        if(!m_isPaused)
        {
            switch(m_LogicState)
            {
            case LOGIC1:
                logic1Process();
                break;
            case LOGIC2:
                logic2Process();
                break;
            case LOGIC3:
                logic3Process();
                break;
            case DEFSTATE:
                defaultProcess();
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
    }
    m_isStarted = false;
}

void ForceLogicIO::pause()
{
    m_isPaused = true;
}

void ForceLogicIO::keepon()
{
    m_isPaused = false;
}

//template <class T>
//bool ForceLogicIO::registStatsandFunc(T _t, FUNC _f)
//{
    
//}

bool ForceLogicIO::getIO(int _IO)
{
    return true;
}

void ForceLogicIO::setIO(int _IO)
{
   // return true;
}

bool ForceLogicIO::getState()
{
    std::lock_guard<std::mutex> lk_gd(m_mutex);
    m_isTargetReach = getIO(REACH_IN);
    m_isTargetReady = getIO(READY_IN);
    return m_isBusy;
}

void ForceLogicIO::logic1Process()
{
    //TODO:移动到抓取位置，等待目标准备完毕
    printf("Process NO:1...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds (1000));
    if(m_isAuto) m_LogicState = LOGIC2;
}
void ForceLogicIO::logic2Process()
{
    //TODO:抓取目标
    printf("Process NO:2...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds (1000));
    if(m_isAuto) m_LogicState = LOGIC3;
}
void ForceLogicIO::logic3Process()
{
    //TODO:目标以获取移动到打磨位置，控制权交给打磨程序
    //if(m_isAuto) m_LogicState = LOGIC1;
    printf("Process NO:3...\n");
    m_isStarted = false;
}
void ForceLogicIO::defaultProcess()
{
    //TODO:返回home位置
    printf("Process NO:default...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds (1000));
    if(m_isAuto) m_LogicState = LOGIC1;
}
