//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_URSTATUSWATCHER_H
#define PROJECT_URSTATUSWATCHER_H

#include <QThread>
#include <condition_variable>

class UrStatusWatcher : public QThread {
Q_OBJECT
public:
    UrStatusWatcher(const std::string& status_type, std::condition_variable& msg_cond);
    virtual ~UrStatusWatcher();

protected:
    virtual void run();

protected:
    std::condition_variable& m_msg_cond;
    std::string m_status_type;
};


#endif //PROJECT_URSTATUSWATCHER_H
