//
// Created by 潘绪洋 on 17-3-31.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTUR_H
#define PROJECT_COBOTUR_H

#include <vector>
#include <stdint.h>
#include <cstdio>

#include <mutex>

class CobotUr {
public:
    static const int MAX_SOCKET_WAIT_ = 1000;
    static const size_t JOINT_NUM_ = 6;
};

class ref_num {
public:
    ref_num() {
        num_ = 0;
    }

    ~ref_num() {
    }

    ref_num& operator++() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        num_++;
        return *this;
    }

    ref_num& operator--() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        num_--;
        return *this;
    }

    void add_ref() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        num_++;
    }

    void dec_ref() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        num_--;
    }


    uint32_t isZero() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        return num_ == 0;
    }

    uint32_t getValue() {
        std::lock_guard<std::mutex> lockGuard(mutex_);
        return num_;
    }

protected:
    std::mutex mutex_;
    uint32_t num_;
};


#endif //PROJECT_COBOTUR_H
