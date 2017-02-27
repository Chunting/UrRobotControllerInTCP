//
// Created by 潘绪洋 on 17-2-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <iostream>
#include <shared_memory.h>
#include <shared_cv_mat.h>
#include <cv.h>

using namespace cobotsys::common;

int main(){

    cv::Mat m;
    BasicSharedCvMat basicSharedCvMat;
    if (basicSharedCvMat.openSharedMat("shared")) {
        basicSharedCvMat.readMat(m);
        cv::pyrDown(m, m);
        cv::imshow("ss", m);
    }
    cv::waitKey(0);
    return 0;
}