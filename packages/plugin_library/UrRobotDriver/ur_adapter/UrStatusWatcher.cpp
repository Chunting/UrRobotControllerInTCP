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
    while (1) {
        m_msg_cond.wait(lck);
        COBOT_LOG.info() << "Status updated, " << m_status_type;
    }
}
