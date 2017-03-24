//
// Created by 潘绪洋 on 17-3-16.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "UrStatusWatcher2.h"
#include <UrAdapterWithIK.h>
#include <eigen3/Eigen/Geometry>
#include <extra2.h>

UrStatusWatcher2::UrStatusWatcher2(UrAdapterWithIK& adpater, const std::string& status_type,
                                   std::condition_variable& msg_cond)
        : QThread(nullptr), m_msg_cond(msg_cond), m_adapter(adpater){
    m_status_type = status_type;
    m_moveitWrapper.loadRobotModel();

    m_obj = std::make_shared<simple_debug_gl_object>();
    m_easyRender.AddRenderer(m_obj);
    m_easyRender.show();
}

UrStatusWatcher2::~UrStatusWatcher2(){
    m_loop = false;
    if (isRunning())
        wait();
    INFO_DESTRUCTOR(this);
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

            Eigen::Affine3d end_effector_state;
            m_moveitWrapper.forwardKinematics(q_actual, end_effector_state);
            auto transform = end_effector_state.rotation();
            Matrix4 tr(end_effector_state.data());

            m_obj->m_transform = tr;
        }

        m_time_last_status = cur_time_point;
    }
}
