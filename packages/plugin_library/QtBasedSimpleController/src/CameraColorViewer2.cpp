//
// Created by 潘绪洋 on 17-3-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CameraColorViewer2.h"
#include <QFile>
#include <QFileDialog>
#include <extra2.h>

CameraColorViewer2::CameraColorViewer2(){
    ui.setupUi(this);

    connect(ui.button_LoadJSON, &QPushButton::released, this, &CameraColorViewer2::loadJSON);
    connect(ui.button_Start, &QPushButton::released, this, &CameraColorViewer2::start);
    connect(ui.button_Stop, &QPushButton::released, this, &CameraColorViewer2::stop);
    connect(ui.button_Pause, &QPushButton::released, this, &CameraColorViewer2::pause);

    m_captureTimer = new QTimer(this);
    m_captureTimer->setInterval(10);
    connect(m_captureTimer, &QTimer::timeout, this, &CameraColorViewer2::captureNew);

    m_camera = nullptr;
}

CameraColorViewer2::~CameraColorViewer2(){
    stop();
    INFO_DESTRUCTOR(this);
}

bool CameraColorViewer2::setup(const QString& configFilePath){
    return true;
}

bool CameraColorViewer2::start(){
    if (m_camera) {
        if (m_camera->open()) {
            m_captureTimer->start();
            return true;
        }
    }
    return false;
}

void CameraColorViewer2::pause(){
    stop();
}

void CameraColorViewer2::stop(){
    m_captureTimer->stop();
    if (m_camera) {
        m_camera->close();
    }
}

void CameraColorViewer2::onCameraStreamUpdate(const std::vector<cobotsys::CameraStreamObserver::StreamFrame>& frames){
    for (const auto& frame : frames) {
        if (frame.frame.type == cobotsys::CameraFrameType::Color) {

            cv::Mat mat;
            cv::pyrDown(frame.frame.image, mat);
            auto image = matToQImage(mat);

            ui.label_Color->setPixmap(QPixmap::fromImage(image));
        }
    }
}

void CameraColorViewer2::loadJSON(){
    QString filename = QFileDialog::getOpenFileName(this, "", "../../data");
    if (filename.isEmpty())
        return;;

    QJsonObject jsonObject;
    if (loadJson(jsonObject, filename.toLocal8Bit().constData())) {
        cobotsys::ObjectGroup objectGroup;
        if (objectGroup.init(jsonObject)) {
            initWidgetView(objectGroup);
        }
    }
}

void CameraColorViewer2::initWidgetView(cobotsys::ObjectGroup& objectGroup){
    auto pCamera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(objectGroup.getObject("camera"));
    if (pCamera == nullptr)
        return;

    pCamera->attach(std::dynamic_pointer_cast<cobotsys::CameraStreamObserver>(shared_from_this()));

    m_objectGroup = objectGroup;
    m_camera = pCamera;
    start();
}

void CameraColorViewer2::captureNew(){
    if (m_camera) {
        m_camera->capture();
    }
}
