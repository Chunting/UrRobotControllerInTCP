//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "easy_cv_mat_publisher.h"
#include "easy_gui.h"

using namespace cobotsys::common;

int main(){
    cv::Mat m = cv::imread("test10.jpg");
    cv::Mat sss;

    cv::pyrDown(m, m);
    std::vector<cv::Mat> mmm, nnn;
    cv::split(m, mmm);
    sss = m.clone();
    int n = 0;


    while (1) {
        sss = m.clone();
        nnn.clear();
        nnn.push_back(mmm[n % 3]);
        nnn.push_back(mmm[(n + 1) % 3]);
        nnn.push_back(mmm[(n + 2) % 3]);
        cv::merge(nnn, m);

        cobotsys::easy_gui_show("Video", m);

        auto mm = sss(cv::Rect(10, 10, 100, 200));

        cobotsys::easy_gui_show("Video2", mm);
        std::cout << "write img: " << n++ << std::endl;

        if (27 == (char) cv::waitKey(0))
            break;
    }
    return 0;
}