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

    BasicSharedCvMat basicSharedCvMat;

    auto imSHOW = [=](const cv::Mat &m){
        cv::Mat nnn;
        cv::pyrDown(m, nnn);
        cv::imshow("ss", nnn);
    };

    int n = 0;
    basicSharedCvMat.setMatUpdatedCallback(imSHOW);
    if (basicSharedCvMat.openSharedMat("shared")) {
        while (1) {
            if (!basicSharedCvMat.readLoop())
                break;
            std::cout << "get a img " << n++ << std::endl;
            if ((char) cv::waitKey(1) == 27)
                break;
        }
    }

    return 0;
}
