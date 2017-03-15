//
// Created by 潘绪洋 on 17-3-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_CAMERACOLORVIEWER_H
#define PROJECT_CAMERACOLORVIEWER_H

#include <QObject>
#include <cobotsys_abstract_object.h>
#include <cobotsys_abstract_controller.h>
#include <cobotsys_file_finder.h>
#include <cobotsys_abstract_camera.h>
#include <QtCore/QTimer>

class CameraColorViewer : public QObject, public cobotsys::AbstractController, public cobotsys::CameraStreamObserver {
Q_OBJECT
public:
    CameraColorViewer();
    ~CameraColorViewer();

    virtual bool start();
    virtual void pause();
    virtual void stop();

    virtual bool setup(const QString& configFilePath);

public:
    virtual void onCameraStreamUpdate(const std::vector<StreamFrame>& frames);

    void updateCamera();
protected:
    std::shared_ptr<cobotsys::AbstractCamera> m_camera;
    QTimer* m_captureTimer;
};


#endif //PROJECT_CAMERACOLORVIEWER_H
