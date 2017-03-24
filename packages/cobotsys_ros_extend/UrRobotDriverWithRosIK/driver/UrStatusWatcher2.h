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
#include <cobotsys_simple_world_renderer_widget.h>
#include <simple_debug_gl_object.h>
#include <QtCore/QSemaphore>

class UrAdapterWithIK;
class UrStatusWatcher2 : public QThread {
Q_OBJECT
public:
    UrStatusWatcher2(UrAdapterWithIK& adpater, const std::string& status_type, std::condition_variable& msg_cond);
    virtual ~UrStatusWatcher2();

    void quitThread();
protected:
    virtual void run();

    void onFinished();
public:
    bool m_loop;
    std::condition_variable& m_msg_cond;
    std::string m_status_type;
    std::chrono::high_resolution_clock::time_point m_time_last_status;
    UrAdapterWithIK& m_adapter;
    ros_moveit_wrapper m_moveitWrapper;
    cobotsys::SimpleWorldRendererWidget m_easyRender;
    std::shared_ptr<simple_debug_gl_object> m_obj;
    QSemaphore m_quitSignal;
};


#endif //PROJECT_URSTATUSWATCHER_H
