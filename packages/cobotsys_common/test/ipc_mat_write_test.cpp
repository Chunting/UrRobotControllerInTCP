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

    cv::Mat m = cv::imread("test10.jpg");
    cv::Mat sss;

    BasicSharedCvMat basicSharedCvMat;
    if (!basicSharedCvMat.createSharedMat("shared", m)) {
        return 1;
    }

    cv::pyrDown(m, sss);
    cv::imshow("src", sss);

    auto push_img = [&](){
        basicSharedCvMat.writeMat(m);
    };

    int n = 0;
    while (1) {
        push_img();
        std::cout << "write img: " << n++ << std::endl;

        if (27 == (char) cv::waitKey(0))
            break;
    }
    return 0;
}