//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "include/easy_cv_mat_reader.h"


namespace cobotsys {
namespace common {


EasyCvMatReader::EasyCvMatReader() :
        easy_shared_names_(EASY_CV_MAT_RW_NAMESPACE) {
}

EasyCvMatReader::~EasyCvMatReader() {
}


std::shared_ptr<EasyCvMatHolder> EasyCvMatReader::lanuch(
        const std::string& img_desc,
        std::function<void(EasyCvMatReaderStatus)> on_status) {

    std::shared_ptr<EasyCvMatHolder> rholder;

    work_mutex_.lock();

    rholder = std::make_shared<EasyCvMatHolder>();
    rholder->name() = img_desc;
    if (easy_shared_names_.hasName(img_desc)) {

        std::thread reader([=]() {
            backgroundWorker(img_desc, rholder, on_status);
        });
        reader.detach();
    } else {
        std::thread reader([=]() {
            if (on_status) {
                on_status(EasyCvMatReaderStatus::TargetImageDoesNotExist);
            } else {
                STD_CERR << "No name for target image" << std::endl;
            }
        });
        reader.detach();
    }

    work_mutex_.unlock();

    return rholder;
}

void EasyCvMatReader::backgroundWorker(const std::string& img_desc,
                                       std::shared_ptr<EasyCvMatHolder> holder,
                                       std::function<void(EasyCvMatReaderStatus)> on_status) {
    BasicSharedCvMat basicSharedCvMat;
    EasyCvMatReaderStatus reader_status;
    cv::Mat local_mat;
    if (basicSharedCvMat.openSharedMat(img_desc)) {
        reader_status = EasyCvMatReaderStatus::ReadSharedCvMatReady;
        if (on_status)
            on_status(reader_status);

        while (true) {
            basicSharedCvMat.wait();

            if (!basicSharedCvMat.readMat(local_mat)) {
                reader_status = EasyCvMatReaderStatus::ImageWriterAlreadyReleased;
                break;
            } // 发送者已经退出

            if (holder->updateMat(local_mat)) {
                if (on_status)
                    on_status(EasyCvMatReaderStatus::MatUpdated);
            } else {
                reader_status = EasyCvMatReaderStatus::ReaderRequireRelease;
                break;
            }
        }
    } else {
        reader_status = EasyCvMatReaderStatus::TargetImageDoesNotExist;
    }

    // print first
    std::cout << "EasyCvMatReader::backgroundWorker finished." << std::endl;

    // final exit notify
    holder->is_reader_released_ = true;

    if (on_status && (!holder->_stop_reader))
        on_status(reader_status);
}


EasyCvMatHolder::EasyCvMatHolder() :
        ready_semaphore_(0) {
    num_post_ = 0;
    num_read_ = 0;
    _stop_reader = false;
    is_reader_released_ = false;
}

bool EasyCvMatHolder::getMat(cv::Mat& mat) {
    if (ready_semaphore_.try_wait()) {
        bool isNew = false;

        num_lock_.lock();
        isNew = (num_post_ != num_read_);
        num_read_ = num_post_;
        num_lock_.unlock();

        mat = inner_mat_;
        return isNew;
    }
    return false;
}

bool EasyCvMatHolder::updateMat(const cv::Mat& mat) {
    if (_stop_reader)
        return true;

    inner_mat_ = mat;

    num_lock_.lock();
    num_post_++;
    num_lock_.unlock();

    bool bresult = true;
    if (callback_)
        bresult = callback_(name(), inner_mat_);

    ready_semaphore_.post();
    return bresult;
}

void EasyCvMatHolder::setUpdateCallback(std::function<bool(const std::string&, const cv::Mat&)> callback) {
    callback_ = callback;
}

void EasyCvMatHolder::stopReader() {
    _stop_reader = true;
}

std::vector<std::string> EasyCvMatReader::existMatNames() {
    return easy_shared_names_.getNames();
}


//

}
}