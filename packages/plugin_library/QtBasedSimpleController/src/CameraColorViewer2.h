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
#include <QCloseEvent>

using namespace cobotsys;


class ImageCache {
public:
    void updateImage(const cv::Mat& image);

    QPixmap getPixmap();
protected:
    std::mutex m_mutex;
    QImage m_image;
};


class CameraColorViewer2 : public AbstractWidget, public CameraStreamObserver {
Q_OBJECT
public:
    CameraColorViewer2();
    virtual ~CameraColorViewer2();

    virtual bool setup(const QString& configFilePath);

    void create();
    void pauseStart();

Q_SIGNALS:
    void imageUpdated();

public:
    void captureNew();
    void updateLabelImage();


    void initCreateList();

public:
    virtual void onCameraStreamUpdate(const cobotsys::CameraFrame& frames, cobotsys::AbstractCamera* camera);

protected:
    virtual void closeEvent(QCloseEvent* event);
protected:


    Ui::CameraColorViewer2 ui;
    std::shared_ptr<AbstractCamera> m_camera;
    QTimer* m_captureTimer;
    ImageCache m_imageCache;
};


#endif //PROJECT_CAMERACOLORVIEWER2_H
