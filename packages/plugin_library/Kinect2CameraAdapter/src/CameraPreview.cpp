//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CameraPreview.h"


CameraPreview::CameraPreview(){
}

CameraPreview::~CameraPreview(){
}

void CameraPreview::onCameraStreamUpdate(const std::vector<cobotsys::CameraStreamObserver::StreamFrame>& frames){
    auto frameCount = frames.size();
    if (frameCount == 0)
        return;;

    for (auto& frame : frames) {
        cv::Mat pdmat;
        if (frame.frame.image.cols * 2 >= 1920)
            cv::pyrDown(frame.frame.image, pdmat);
        else
            pdmat = frame.frame.image;

        cv::imshow(frame.frame.typeName(), pdmat);
    }
}
