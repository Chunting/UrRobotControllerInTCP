//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_KINECTCAMERADETECTOR_H
#define PROJECT_KINECTCAMERADETECTOR_H

#include <QObject>
#include <QTimer>
#include <memory>

class KinectCameraDetector : public QObject {
Q_OBJECT
public:
    KinectCameraDetector(QObject* parent = nullptr);
    ~KinectCameraDetector();


    void startCheck();

Q_SIGNALS:
    void cameraFound(bool has_camera_connected);


protected:
    void checkCameraDeviceNum();


protected:
    class KinectCameraDetectorImpl;

    std::shared_ptr<KinectCameraDetectorImpl> m_impl;
    QTimer* m_timer;
    int m_dev_num;

};


#endif //PROJECT_KINECTCAMERADETECTOR_H
