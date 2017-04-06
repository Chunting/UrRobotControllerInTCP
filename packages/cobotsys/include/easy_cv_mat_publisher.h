//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COMMON_EASY_CV_MAT_PUBLISHER_H
#define COBOTSYS_COMMON_EASY_CV_MAT_PUBLISHER_H

#include "easy_cv_mat_def.h"
#include "shared_cv_mat.h"
#include "easy_shared_names.h"
#include <map>

namespace cobotsys {
namespace common {


class EasyCvMatPublisher {
public:
    EasyCvMatPublisher();
    ~EasyCvMatPublisher();
    bool publishCvMat(const std::string& img_desc, const cv::Mat& img);

protected:
    class CvMatPublishWorker {
    public:
        ~CvMatPublishWorker();
        BasicSharedCvMat shared_cv_mat;


        bool publishCvMat(const cv::Mat& img);
    };

protected:
    std::shared_ptr<CvMatPublishWorker> findOrCreatePostWorker(const std::string& img_desc, const cv::Mat& img);
protected:
    EasySharedNames easy_shared_names_;
    std::map<std::string, std::shared_ptr<CvMatPublishWorker> > easy_publish_workers_;
};
}
}


#endif //COBOTSYS_COMMON_EASY_CV_MAT_PUBLISHER_H
