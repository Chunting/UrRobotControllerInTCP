//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include "easy_cv_mat_publisher.h"
#include "easy_cv_mat_reader.h"

using namespace cobotsys::common;

int main(){

    cv::Mat mm, nn;
    EasyCvMatReader reader;
    auto holder = reader.lanuch("a");


    int n = 0;

    while (1) {
        if (holder->getMat(mm)) {
            cv::pyrDown(mm, nn);
            cv::imshow("aa", nn);
            std::cout << "read img: " << n++ << std::endl;
        }
        if (holder->isReaderReleased())
            break;
        if (27 == (char) cv::waitKey(20))
            break;
    }
    return 0;
}