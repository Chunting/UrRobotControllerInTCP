//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URSTATUSWATCHER_H
#define PROJECT_URSTATUSWATCHER_H

#include <QThread>
#include <condition_variable>
#include <chrono>

class UrAdapter;
class UrStatusWatcher : public QThread {
Q_OBJECT
public:
    UrStatusWatcher(UrAdapter& adpater, const std::string& status_type, std::condition_variable& msg_cond);
    virtual ~UrStatusWatcher();

    void stopWatcher();
protected:
    virtual void run();

protected:
    bool m_loop;
    std::condition_variable& m_msg_cond;
    std::string m_status_type;
    std::chrono::high_resolution_clock::time_point m_time_last_status;
    UrAdapter& m_adapter;
};


#endif //PROJECT_URSTATUSWATCHER_H
