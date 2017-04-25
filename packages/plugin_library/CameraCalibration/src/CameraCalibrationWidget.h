//
// Created by eleven on 17-4-19.
//

#ifndef COBOTSYS_CAMERACALIBRATIONWIDGET_H
#define COBOTSYS_CAMERACALIBRATIONWIDGET_H

#include "ui_CameraCalibrationWidget.h"
#include "CameraCalibration.h"
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

class CameraCalibrationWidget : public AbstractWidget, public CameraStreamObserver {
Q_OBJECT
public:
    CameraCalibrationWidget();
    ~CameraCalibrationWidget();

    virtual bool setup(const QString& configFilePath);

    void create();
    void pauseStart();
    void calibration();
    void registration();

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


    Ui::CameraCalibration ui;
    std::shared_ptr<AbstractCamera> m_camera;
    QTimer* m_captureTimer;
    ImageCache m_imageCache;

    std::shared_ptr<CameraCalibration> m_cameraCalibration;
    bool m_rgbCalibrating;
    bool m_depthCalibrating;
};


#endif //COBOTSYS_CAMERACALIBRATIONWIDGET_H
