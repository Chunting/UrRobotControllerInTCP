//
// Created by 潘绪洋 on 17-2-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COMMON_SHARED_CV_MAT_H
#define COBOTSYS_COMMON_SHARED_CV_MAT_H

#include "shared_memory.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <cv.h>
#include <boost/interprocess/sync/named_semaphore.hpp>
#include <functional>

namespace cobotsys {
namespace common {

class BasicSharedCvMat {
public:
    BasicSharedCvMat();
    ~BasicSharedCvMat();

    bool createSharedMat(const std::string &mat_name, const cv::Mat &mat_info);
    bool openSharedMat(const std::string &mat_name);
    bool openSharedMat(const std::string &mat_name, const cv::Mat &mat_info);
    void setMatUpdatedCallback(std::function<void(const cv::Mat &)> on_mat_updated);

    bool writeMat(const cv::Mat &mat);
    bool readMat(cv::Mat &mat);

    void writeEmptyMat();

    bool readLoop();
    void wait();
protected:
    size_t calcMatSizeUsed(const cv::Mat &mat) const;

protected:
    SharedMemory mat_mem_;

    struct mat_head {
        int rows;
        int cols;
        int type;
        int step;
    };
    std::function<void(const cv::Mat &)> mat_updated_callback_;
};
}
}


#endif //COBOTSYS_COMMON_SHARED_CV_MAT_H
