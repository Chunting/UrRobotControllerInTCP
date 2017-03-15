//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "easy_cv_mat_publisher.h"

namespace cobotsys {
namespace common {


EasyCvMatPublisher::EasyCvMatPublisher() :
        easy_shared_names_(EASY_CV_MAT_RW_NAMESPACE){
}

EasyCvMatPublisher::~EasyCvMatPublisher(){

    // 释放由当前类创建的名字
    for (const auto &iter : easy_publish_workers_) {
        easy_shared_names_.removeName(iter.first);
    }
}

bool EasyCvMatPublisher::publishCvMat(const std::string &img_desc, const cv::Mat &img){
    auto publishWorker = findOrCreatePostWorker(img_desc, img);
    if (publishWorker) {
        return publishWorker->publishCvMat(img);
    }
    return false;
}

std::shared_ptr<EasyCvMatPublisher::CvMatPublishWorker>
EasyCvMatPublisher::findOrCreatePostWorker(const std::string &img_desc, const cv::Mat &img){

    if (easy_shared_names_.hasName(img_desc)) {
        auto iter = easy_publish_workers_.find(img_desc);
        if (iter != easy_publish_workers_.end())
            return iter->second;

        // 给定的参数名字存在，但是不当前进程创建的,
        // 或者是当前进程意外退出，然后重新打开。
        // 无论怎么样，重新打开对应的数据，写入即可。

        auto publishWorker = std::make_shared<CvMatPublishWorker>();
        if (publishWorker->shared_cv_mat.openSharedMat(img_desc, img)) {
            easy_publish_workers_.insert({img_desc, publishWorker});
            return publishWorker;
        }

        return nullptr;
    }

    auto publishWorker = std::make_shared<CvMatPublishWorker>();
    if (publishWorker) {
        if (publishWorker->shared_cv_mat.createSharedMat(img_desc, img)) {
            easy_publish_workers_.insert({img_desc, publishWorker});
            easy_shared_names_.pushName(img_desc);
            return publishWorker;
        }
    }

    return nullptr;
}


bool EasyCvMatPublisher::CvMatPublishWorker::publishCvMat(const cv::Mat &img){
    return shared_cv_mat.writeMat(img);
}

EasyCvMatPublisher::CvMatPublishWorker::~CvMatPublishWorker(){
    shared_cv_mat.writeEmptyMat();
}
}
}