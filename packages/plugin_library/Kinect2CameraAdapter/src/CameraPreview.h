//
// Created by 潘绪洋 on 17-3-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_CAMERAPREVIEW_H
#define PROJECT_CAMERAPREVIEW_H

#include <cobotsys_abstract_camera.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

class CameraPreview : public cobotsys::CameraStreamObserver, public cobotsys::AbstractObject {
public:
    CameraPreview();
    virtual ~CameraPreview();

    virtual void onCameraStreamUpdate(const std::vector<StreamFrame>& frames);

    virtual bool setup(const QString& configFilePath);
};


#endif //PROJECT_CAMERAPREVIEW_H
