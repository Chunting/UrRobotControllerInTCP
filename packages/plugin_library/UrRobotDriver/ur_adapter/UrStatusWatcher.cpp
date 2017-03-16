//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "UrStatusWatcher.h"


UrStatusWatcher::UrStatusWatcher(const std::string& status_type, std::condition_variable& msg_cond)
        : QThread(nullptr), m_msg_cond(msg_cond){
    m_status_type = status_type;
}

UrStatusWatcher::~UrStatusWatcher(){
}

void UrStatusWatcher::run(){
    std::mutex m;
    std::unique_lock<std::mutex> lck(m);

    m_time_last_status = std::chrono::high_resolution_clock::now();
    while (true) {
        m_msg_cond.wait(lck);

        auto cur_time_point = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_diff = cur_time_point - m_time_last_status;
        COBOT_LOG.info() << "Status updated, " << m_status_type
                         << ", time: " << time_diff.count() * 1000 << "ms, "
                         << 1 / time_diff.count() << "hz";


        m_time_last_status = cur_time_point;
    }
}
