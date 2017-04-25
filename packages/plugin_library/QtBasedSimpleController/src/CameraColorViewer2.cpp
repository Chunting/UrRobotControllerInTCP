//
// Created by 潘绪洋 on 17-3-14.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "CameraColorViewer2.h"
#include <QFile>
#include <QFileDialog>
#include <extra2.h>
#include <opencv2/opencv.hpp>
#include <cobotsys_file_finder.h>

CameraColorViewer2::CameraColorViewer2() {
    ui.setupUi(this);

    connect(this, &CameraColorViewer2::imageUpdated, this, &CameraColorViewer2::updateLabelImage);
    connect(ui.btnCreate, &QPushButton::released, this, &CameraColorViewer2::create);
    connect(ui.btnPauseStart, &QPushButton::released, this, &CameraColorViewer2::pauseStart);

    m_captureTimer = new QTimer(this);
    m_captureTimer->setInterval(10);
    connect(m_captureTimer, &QTimer::timeout, this, &CameraColorViewer2::captureNew);

    m_camera = nullptr;
}

CameraColorViewer2::~CameraColorViewer2() {
    INFO_DESTRUCTOR(this);
}

bool CameraColorViewer2::setup(const QString& configFilePath) {
    initCreateList();
    return true;
}


void CameraColorViewer2::onCameraStreamUpdate(const cobotsys::CameraFrame& frames, cobotsys::AbstractCamera* camera) {
    for (const auto& frame : frames.frames) {
        if (frame.type == cobotsys::ImageType::Color) {
            cv::Mat mat;
            if (frame.data.cols > 1920 / 2) {
                cv::pyrDown(frame.data, mat);
            } else {
                mat = frame.data;
            }

            m_imageCache.updateImage(mat);
            Q_EMIT imageUpdated();
        }
    }
}

void CameraColorViewer2::captureNew() {
    if (m_camera) {
        m_camera->capture();
    }
}

void CameraColorViewer2::updateLabelImage() {
    ui.label_Color->setPixmap(m_imageCache.getPixmap());
}

void CameraColorViewer2::create() {
    if (!GlobalObjectFactory::instance()) return;
    if (ui.cboCamera->count() == 0) {
        COBOT_LOG.error() << "No Camera Driver Exist!";
        return;
    }

    QString objConfig = QFileDialog::getOpenFileName(this,
        tr("Get Camera Config JSON file ..."),
        QString(FileFinder::getPreDefPath().c_str()),
        tr("JSON files (*.JSON *.json)"));
    if (objConfig.isEmpty()) {
        COBOT_LOG.warning() << "Camera config is empty, Camera create may fail!";
    }

    QStringList obj_info = ui.cboCamera->currentData().toStringList();
    QString factory = obj_info.at(0);
    QString typen = obj_info.at(1);

    auto obj = GlobalObjectFactory::instance()->createObject(factory, typen);
    m_camera = std::dynamic_pointer_cast<AbstractCamera>(obj);
    if (m_camera) {
        auto ob = std::dynamic_pointer_cast<CameraStreamObserver>(shared_from_this());
        m_camera->attach(ob);
        if (m_camera->setup(objConfig)) {
            if (m_camera->open(0)) {
                COBOT_LOG.notice() << "Create and setup success";
                m_captureTimer->start();
                return;
            } else {
            }
        }
    }
    m_camera.reset();
}

void CameraColorViewer2::initCreateList() {
    if (!GlobalObjectFactory::instance()) return;

    ui.cboCamera->clear();
    auto factory_names = GlobalObjectFactory::instance()->getFactoryNames();
    for (auto& name : factory_names) {
        auto types = GlobalObjectFactory::instance()->getFactorySupportedNames(name);

        for (auto& type : types) {
            auto obj = GlobalObjectFactory::instance()->createObject(name, type);
            auto ptrCamera = std::dynamic_pointer_cast<AbstractCamera>(obj);
            if (ptrCamera) {
                QStringList data;
                QString text;
                text = QString("%1 - %2").arg(name.c_str()).arg(type.c_str());
                data << name.c_str();
                data << type.c_str();
                ui.cboCamera->addItem(text, data);
            }
        }
    }
}

void CameraColorViewer2::pauseStart() {
    if (m_camera) {
        if (m_camera->isOpened()) {
            m_camera->close();
        } else {
            m_camera->open();
        }
    }
}

void CameraColorViewer2::closeEvent(QCloseEvent* event) {
    if (m_camera) {
        m_camera->clearAttachedObject();
        m_camera->close();
    }
    QWidget::closeEvent(event);
}


void ImageCache::updateImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock_guard(m_mutex);

    m_image = matToQImage(image);
}

QPixmap ImageCache::getPixmap() {
    std::lock_guard<std::mutex> lock_guard(m_mutex);
    return QPixmap::fromImage(m_image);
}
