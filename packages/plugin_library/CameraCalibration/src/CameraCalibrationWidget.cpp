//
// Created by eleven on 17-4-19.
//

#include "CameraCalibrationWidget.h"
#include <QFileDialog>
#include <extra2.h>
#include <cobotsys_file_finder.h>

static const int boardSizeWidth = 5;  // 标定板宽（角点列数）
static const int boardSizeHeight = 7;  // 标定板高（角点行数）
static const string caliResultFile = "caliResult1.xml";

CameraCalibrationWidget::CameraCalibrationWidget() {
    ui.setupUi(this);

    connect(this, &CameraCalibrationWidget::imageUpdated, this, &CameraCalibrationWidget::updateLabelImage);
    connect(ui.btnCreate, &QPushButton::released, this, &CameraCalibrationWidget::create);
    connect(ui.btnPauseStart, &QPushButton::released, this, &CameraCalibrationWidget::pauseStart);
    connect(ui.btnCalibration, &QPushButton::released, this, &CameraCalibrationWidget::calibration);
    connect(ui.btnRegistration, &QPushButton::released, this, &CameraCalibrationWidget::registration);


    m_captureTimer = new QTimer(this);
    m_captureTimer->setInterval(10);
    connect(m_captureTimer, &QTimer::timeout, this, &CameraCalibrationWidget::captureNew);

    m_camera = nullptr;
    m_rgbCalibrating = false;
    m_depthCalibrating = false;
}

CameraCalibrationWidget::~CameraCalibrationWidget() {
    INFO_DESTRUCTOR(this);
}

bool CameraCalibrationWidget::setup(const QString& configFilePath) {
    initCreateList();
    return true;
}


void CameraCalibrationWidget::onCameraStreamUpdate(const cobotsys::CameraFrame& frames) {
    for (const auto& frame : frames.frames) {
        if (frame.type == cobotsys::ImageType::Ir) {
            cv::Mat mat = frame.data;

            if (m_depthCalibrating) {
                if (m_cameraCalibration->calibrateFromCamera(mat,
                                                             cv::Size(m_camera->getImageWidth(2), m_camera->getImageHeight(2))))
                    m_depthCalibrating = false;
            }
        }

        if (frame.type == cobotsys::ImageType::Color) {
            cv::Mat mat;
//            if (frame.data.cols > 1920 / 2) {   先不考虑图片宽高问题
//                cv::pyrDown(frame.data, mat);
//            } else {
                mat = frame.data;
//            }

            if (m_rgbCalibrating) {
                //开始标定，开始先寻找角点
                if (m_cameraCalibration->calibrateFromCamera(mat,
                                                             cv::Size(m_camera->getImageWidth(0), m_camera->getImageHeight(0))))
                    //标定成功，则结束标定
                    m_rgbCalibrating = false;
            }

            m_imageCache.updateImage(mat);
            Q_EMIT imageUpdated();
        }
    }

    if (!m_rgbCalibrating && !m_depthCalibrating)
        m_captureTimer->setInterval(10);
}

void CameraCalibrationWidget::captureNew() {
    if (m_camera) {
        m_camera->capture();
    }
}

void CameraCalibrationWidget::updateLabelImage() {
    ui.label_Color->setPixmap(m_imageCache.getPixmap());
}

void CameraCalibrationWidget::create() {
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

void CameraCalibrationWidget::initCreateList() {
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

void CameraCalibrationWidget::pauseStart() {
    if (m_camera) {
        if (m_camera->isOpened()) {
            m_camera->close();
        } else {
            m_camera->open();
        }
    }
}

void CameraCalibrationWidget::closeEvent(QCloseEvent* event) {
    if (m_camera) {
        m_camera->clearAttachedObject();
        m_camera->close();
    }
    QWidget::closeEvent(event);
}

void CameraCalibrationWidget::calibration() {
    //图片宽高
    m_cameraCalibration = std::make_shared<CameraCalibration>(cv::Size(boardSizeWidth, boardSizeHeight), caliResultFile);
    //开始标定，则每两秒抓取一次图像
    m_captureTimer->setInterval(5000);

    //若相机支持彩色图像，则进行RGB标定
    if (m_camera->getImageType(0) == ImageType::Color)
        m_rgbCalibrating = true;

    //若相机支持红外图像，则进行深度标定
    if (m_camera->getImageType(2) == ImageType::Ir)
        m_depthCalibrating = true;
}

void CameraCalibrationWidget::registration() {

}

void ImageCache::updateImage(const cv::Mat& image) {
    std::lock_guard<std::mutex> lock_guard(m_mutex);

    m_image = matToQImage(image);
}

QPixmap ImageCache::getPixmap() {
    std::lock_guard<std::mutex> lock_guard(m_mutex);
    return QPixmap::fromImage(m_image);
}
