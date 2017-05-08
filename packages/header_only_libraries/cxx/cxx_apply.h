//
// Created by 潘绪洋 on 17-5-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_CXX_APPLY_H_H
#define COBOTSYS_CXX_APPLY_H_H

#include <vector>
#include <cmath>

template<class T>
void scale_all(std::vector<T>& vec_, double s_) {
    for (auto& iter : vec_) {
        iter = (T) (iter * s_);
    }
}

template<class T>
T distance(const std::vector<T>& vec_) {
    T sum_ = 0;
    for (auto& iter : vec_) {
        sum_ += iter * iter;
    }
    return std::sqrt(sum_);
}

template<class T, class M>
void limit(T& val_, const M& max_) {
    T abs_m = (T) std::fabs(max_);
    T abs_n = -abs_m;

    if (val_ > abs_m)
        val_ = abs_m;
    if (val_ < abs_n)
        val_ = abs_n;
};

template<class T>
void add_num(std::vector<T>& vec_, double num_) {
    for (auto& iter : vec_) {
        iter += (T) num_;
    }
}

template<class T>
void add_other(std::vector<T>& vec_, const std::vector<T>& oth_) {
    if (vec_.size() == oth_.size()) {
        for (size_t i = 0; i < vec_.size(); i++) {
            vec_[i] += oth_[i];
        }
    }
}

template<class T>
void norm_all(std::vector<T>& vec_) {
    for (auto& iter : vec_) {
        iter = std::fabs(iter);
    }
}

#endif //COBOTSYS_CXX_APPLY_H_H
