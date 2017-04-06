//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EASY_GUI_H
#define PROJECT_EASY_GUI_H

#include <string>
#include <cv.h>

namespace cobotsys {

void easy_gui_reset();
void easy_gui_show_setup(bool show_with_cv_imshow = true);
void easy_gui_show(const std::string& title, const cv::Mat& image);
}

#endif //PROJECT_EASY_GUI_H
