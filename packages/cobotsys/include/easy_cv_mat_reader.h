//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COMMON_EASY_CV_MAT_READER_H
#define COBOTSYS_COMMON_EASY_CV_MAT_READER_H

#include "easy_cv_mat_def.h"
#include "shared_cv_mat.h"
#include "easy_shared_names.h"
#include <map>
#include <thread>

namespace cobotsys {
namespace common {

enum class EasyCvMatReaderStatus {
    MatUpdated,
    TargetImageDoesNotExist,
    ImageWriterAlreadyReleased,
    ReaderRequireRelease,
    ReadSharedCvMatReady,
};

class EasyCvMatReader;
class EasyCvMatHolder {
public:
    EasyCvMatHolder();

    bool getMat(cv::Mat &mat);
    bool updateMat(const cv::Mat &mat);
    void setUpdateCallback(std::function<bool(const std::string &, const cv::Mat &)> callback);

    bool isReaderReleased(){ return is_reader_released_; }

    std::string &name(){ return mat_name_; }

    const std::string &name() const{ return mat_name_; }

    void stopReader();

    friend class EasyCvMatReader;
protected:
    boost::interprocess::interprocess_semaphore ready_semaphore_;
    boost::interprocess::interprocess_mutex num_lock_;
    std::function<bool(const std::string &, const cv::Mat &)> callback_;
    cv::Mat inner_mat_;
    uint32_t num_post_;
    uint32_t num_read_;
    bool is_reader_released_;
    std::string mat_name_;

    bool _stop_reader;
};


class EasyCvMatReader {
public:
    EasyCvMatReader();
    ~EasyCvMatReader();

    std::vector<std::string> existMatNames();

    std::shared_ptr<EasyCvMatHolder> lanuch(const std::string &img_desc,
                                            std::function<void(EasyCvMatReaderStatus reason)> on_status = nullptr);

    friend class EasyCvMatHolder;
protected:
    void backgroundWorker(const std::string &img_desc,
                          std::shared_ptr<EasyCvMatHolder> holder,
                          std::function<void(EasyCvMatReaderStatus)> on_status);
protected:
    EasySharedNames easy_shared_names_;
    boost::interprocess::interprocess_mutex work_mutex_;
};



//
}
}

#endif //COBOTSYS_COMMON_EASY_CV_MAT_READER_H
