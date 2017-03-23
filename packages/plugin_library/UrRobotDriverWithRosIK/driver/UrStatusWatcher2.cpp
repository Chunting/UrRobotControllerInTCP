//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "UrStatusWatcher2.h"
#include <UrAdapterWithIK.h>

UrStatusWatcher2::UrStatusWatcher2(UrAdapterWithIK& adpater, const std::string& status_type,
                                   std::condition_variable& msg_cond)
        : QThread(nullptr), m_msg_cond(msg_cond), m_adapter(adpater){
    m_status_type = status_type;
}

UrStatusWatcher2::~UrStatusWatcher2(){
}

void UrStatusWatcher2::run(){
    std::mutex m;
    std::unique_lock<std::mutex> lck(m);

    m_loop = true;


    m_time_last_status = std::chrono::high_resolution_clock::now();
    while (m_loop) {
        m_msg_cond.wait(lck);

        auto cur_time_point = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_diff = cur_time_point - m_time_last_status;

        auto& driver = m_adapter.getDriver();

        if (driver) {
            auto q_actual = driver->rt_interface_->robot_state_->getQActual();
            m_adapter.notify([=](std::shared_ptr<RobotStatusObserver>& observer){
                observer->onJointStatusUpdate(q_actual);
            });
        }

        m_time_last_status = cur_time_point;
    }
}
