//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URSTATUSWATCHER_H
#define PROJECT_URSTATUSWATCHER_H

#include <QThread>
#include <condition_variable>
#include <chrono>
#include <ros_moveit_wrapper.h>

class UrAdapterWithIK;
class UrStatusWatcher2 : public QThread {
Q_OBJECT
public:
    UrStatusWatcher2(UrAdapterWithIK& adpater, const std::string& status_type, std::condition_variable& msg_cond);
    virtual ~UrStatusWatcher2();

protected:
    virtual void run();

protected:
    bool m_loop;
    std::condition_variable& m_msg_cond;
    std::string m_status_type;
    std::chrono::high_resolution_clock::time_point m_time_last_status;
    UrAdapterWithIK& m_adapter;
    ros_moveit_wrapper m_moveitWrapper;
};


#endif //PROJECT_URSTATUSWATCHER_H
