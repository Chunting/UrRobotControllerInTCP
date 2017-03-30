//
// Created by 潘绪洋 on 17-2-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "include/shared_cv_mat.h"

namespace cobotsys {
namespace common {

BasicSharedCvMat::BasicSharedCvMat(){
}

BasicSharedCvMat::~BasicSharedCvMat(){
}

bool BasicSharedCvMat::createSharedMat(const std::string &mat_name, const cv::Mat &mat_info){
    if (mat_mem_.create(mat_name, calcMatSizeUsed(mat_info))) {
        return true;
    }
    return false;
}

bool BasicSharedCvMat::openSharedMat(const std::string &mat_name){
    if (mat_mem_.open(mat_name)) {
        return true;
    }
    return false;
}


bool BasicSharedCvMat::openSharedMat(const std::string &mat_name, const cv::Mat &mat_info){
    if (mat_mem_.open(mat_name, calcMatSizeUsed(mat_info))) {
        return true;
    }
    return false;
}

bool BasicSharedCvMat::writeMat(const cv::Mat &mat){
    auto mat_data_len = calcMatSizeUsed(mat);
    auto mat_head_len = sizeof(mat_head);
    auto mat_body_len = mat_data_len - mat_head_len;

    mat_head mathdr;
    mathdr.rows = mat.rows;
    mathdr.cols = mat.cols;
    mathdr.type = mat.type();
    mathdr.step = (int)mat.step[0];
    auto mmsize = mat_mem_.size();
    if (mat_data_len == mmsize && mat_mem_.isValid()) {
        mat_mem_.lock();
        mat_mem_.write(mathdr);
        mat_mem_.write(mat.data, mat_body_len, mat_head_len);
        mat_mem_.unlock();
        mat_mem_.getSemaphore().post();
        return true;
    }
    return false;
}

bool BasicSharedCvMat::readMat(cv::Mat &mat){
    mat_head mathdr;
    auto mat_head_len = sizeof(mat_head);

    bool bresult = false;
    if (mat_mem_.isValid()) {
        mat_mem_.lock();
        mat_mem_.read(&mathdr, mat_head_len);
        if (mathdr.cols * mathdr.rows) {
            mat = cv::Mat(mathdr.rows, mathdr.cols, mathdr.type, mat_mem_.rawDataAt(mat_head_len), mathdr.step).clone();
            bresult = true;
        }
        mat_mem_.unlock();
    }
    return bresult;
}

size_t BasicSharedCvMat::calcMatSizeUsed(const cv::Mat &mat) const{
    return (size_t) (mat.step[0] * mat.rows) + sizeof(mat_head);
}

void BasicSharedCvMat::setMatUpdatedCallback(std::function<void(const cv::Mat &)> on_mat_updated){
    mat_updated_callback_ = on_mat_updated;
}

bool BasicSharedCvMat::readLoop(){
    mat_mem_.getSemaphore().wait();

    if (mat_updated_callback_) {
        cv::Mat m;
        if (readMat(m)) {
            mat_updated_callback_(m);
            return true;
        }
    }

    return false;
}

void BasicSharedCvMat::writeEmptyMat(){
    mat_head mathdr = {0};
    auto mmsize = mat_mem_.size();
    if (mat_mem_.isValid()) {
        mat_mem_.lock();
        mat_mem_.write(mathdr);
        mat_mem_.unlock();
        mat_mem_.getSemaphore().post();
    }
}

void BasicSharedCvMat::wait(){
    mat_mem_.getSemaphore().wait();
}
}
}

