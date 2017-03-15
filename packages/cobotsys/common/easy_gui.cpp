//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "easy_gui.h"
#include "easy_cv_mat_publisher.h"
#include "cobotsys_logger.h"

namespace cobotsys {


namespace easy_gui_alpha_impl {
static bool show_with_cv_imshow = true;

common::EasyCvMatPublisher &publisher(){
    static common::EasyCvMatPublisher publisher_;
    return publisher_;
}


struct EasyGUIShowHint {
    int width;
    int height;
    int type;

    EasyGUIShowHint(){
        width = 0;
        height = 0;
        type = 0;
    }

    void collectAttr(const cv::Mat &image){
        width = image.cols;
        height = image.rows;
        type = image.type();
    }

    bool equal(const cv::Mat &image) const{
        EasyGUIShowHint hint;
        hint.collectAttr(image);
        return (width == hint.width) && (height == hint.height) && (type == hint.type);
    }
};

void check_pub_mat(const std::string &title, const cv::Mat &image){
    static std::map<std::string, EasyGUIShowHint> hints;

    auto iter = hints.find(title);
    if (iter != hints.end()) {
        auto &hint = iter->second;
        if (!hint.equal(image)) {
            COBOT_LOG.error() << "Image for <" << title << "> is different with first";
        }
    } else {
        auto &hint = hints[title];
        hint.collectAttr(image);
    }
}
}

void easy_gui_show(const std::string &title, const cv::Mat &image){
    if (image.isContinuous()) {
        easy_gui_alpha_impl::check_pub_mat(title, image);
        easy_gui_alpha_impl::publisher().publishCvMat(title, image);
    } else {
        auto clonedMat = image.clone();
        easy_gui_alpha_impl::check_pub_mat(title, clonedMat);
        easy_gui_alpha_impl::publisher().publishCvMat(title, clonedMat);
    }

    if (easy_gui_alpha_impl::show_with_cv_imshow)
        cv::imshow(title, image);
}

void easy_gui_reset(){
    common::EasySharedNames::dumpAllNames(std::cout, EASY_CV_MAT_RW_NAMESPACE);
    common::EasySharedNames::remove(EASY_CV_MAT_RW_NAMESPACE);
    common::EasySharedNames::dumpAllNames(std::cout, EASY_CV_MAT_RW_NAMESPACE);
}

void easy_gui_show_setup(bool show_with_cv_imshow){
    easy_gui_alpha_impl::show_with_cv_imshow = show_with_cv_imshow;
}
}