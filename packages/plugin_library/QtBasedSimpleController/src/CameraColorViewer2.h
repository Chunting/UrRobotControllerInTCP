//
// Created by 潘绪洋 on 17-3-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_CAMERACOLORVIEWER2_H
#define PROJECT_CAMERACOLORVIEWER2_H

#include "ui_CameraColorViewer2.h"
#include <cobotsys_abstract_controller.h>
#include <cobotsys_abstract_camera.h>
#include <QPushButton>
#include <QTimer>


class ImageCache {
public:
    void updateImage(const cv::Mat& image);

    QPixmap getPixmap();
protected:
    std::mutex m_mutex;
    QImage m_image;
};


class CameraColorViewer2
        : public cobotsys::AbstractControllerWidget, public cobotsys::CameraStreamObserver {
Q_OBJECT
public:
    CameraColorViewer2();
    virtual ~CameraColorViewer2();

    virtual bool setup(const QString& configFilePath);

    virtual bool start();
    virtual void pause();
    virtual void stop();


Q_SIGNALS:
    void imageUpdated();

public:
    void loadJSON();
    void captureNew();
    void updateLabelImage();

public:
    virtual void onCameraStreamUpdate(const std::vector<StreamFrame>& frames);

protected:
    void initWidgetView(cobotsys::ObjectGroup& objectGroup);

protected:
    Ui::CameraColorViewer2 ui;
    cobotsys::ObjectGroup m_objectGroup;
    std::shared_ptr<cobotsys::AbstractCamera> m_camera;
    QTimer* m_captureTimer;
    ImageCache m_imageCache;
};


#endif //PROJECT_CAMERACOLORVIEWER2_H
