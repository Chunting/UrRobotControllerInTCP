//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <extra2.h>
#include "cobotsys_abstract_camera.h"


namespace cobotsys {
AbstractCamera::AbstractCamera() {
}

AbstractCamera::~AbstractCamera() {
    INFO_DESTRUCTOR(this);
}

CameraStreamObserver::CameraStreamObserver() {
}

CameraStreamObserver::~CameraStreamObserver() {
    INFO_DESTRUCTOR(this);
}
}
