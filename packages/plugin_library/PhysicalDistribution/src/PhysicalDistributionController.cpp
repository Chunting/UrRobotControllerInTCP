//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include <cobotsys_file_finder.h>
#include "PhysicalDistributionController.h"
#include "RobotStatusViewer.h"


PhysicalDistributionController::PhysicalDistributionController() {
    m_robotConnected = false;
    m_loop = false;
    m_jsonServer = new JsonServer(this);
    connect(m_jsonServer, &JsonServer::reqStart, this, &PhysicalDistributionController::start);
    connect(m_jsonServer, &JsonServer::reqStop, this, &PhysicalDistributionController::stop);

    setupUi();
}

PhysicalDistributionController::~PhysicalDistributionController() {
    removeSharedWidget(m_ptrViewer);
    removeSharedWidget(m_ptrManiputor);
    detachSharedObject(m_ptrRobot);
    detachSharedObject(m_ptrCameraMaster);
    detachSharedObject(m_ptrKinematicSolver);
    detachSharedObject(m_ptrMover);
    detachSharedObject(m_ptrDetector);
    detachSharedObject(m_ptrPicker);
    detachSharedObject(m_ptrPlacer);
    detachSharedObject(m_ptrManiputor);
}

bool PhysicalDistributionController::setup(const QString& configFilePath) {
    ObjectGroup objectGroup;
    QJsonObject jsonObject;
    if (loadJson(jsonObject, configFilePath)) {
        if (objectGroup.init(jsonObject)) {
            return _setupInternalObjects(objectGroup);
        }
    }
    COBOT_LOG.error() << "fail to load json config.";
    return false;
}

bool PhysicalDistributionController::start() {
    bool success = false;
    if (m_ptrRobot) {
        success = m_ptrRobot->start();
        if (success) {
            success = m_ptrCameraMaster->open();
            if (success) {
                m_numImageCaptured = 0;
                m_loop = true;
                m_mainTaskThread = std::thread(&PhysicalDistributionController::mainLoop, this);
            }
        }
    }
    return success;
}

void PhysicalDistributionController::pause() {
    stop();
}

void PhysicalDistributionController::stop() {
    m_loop = false;
    m_cond.notify_all();
    if (m_mainTaskThread.joinable()) {
        m_mainTaskThread.join();
    }

    if (m_ptrRobot) {
        m_ptrRobot->stop();
    }
    if (m_ptrCameraMaster) {
        m_ptrCameraMaster->close();
    }
}

void PhysicalDistributionController::onArmRobotConnect() {
    std::lock_guard<std::mutex> lockctx(m_mutex);
    m_robotConnected = true;
}

void PhysicalDistributionController::onArmRobotDisconnect() {
    std::lock_guard<std::mutex> lockctx(m_mutex);
    m_robotConnected = false;
}

void PhysicalDistributionController::onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus) {
}

using std::dynamic_pointer_cast;

bool PhysicalDistributionController::_setupInternalObjects(ObjectGroup& objectGroup) {
    m_ptrRobot = dynamic_pointer_cast<AbstractArmRobotRealTimeDriver>(objectGroup.getObject("Robot"));
    m_ptrKinematicSolver = dynamic_pointer_cast<AbstractKinematicSolver>(objectGroup.getObject("KinematicSolver"));
    m_ptrMover = dynamic_pointer_cast<AbstractArmRobotMoveDriver>(objectGroup.getObject("RobotMover"));
    m_ptrCameraMaster = dynamic_pointer_cast<AbstractCamera>(objectGroup.getObject("MasterCamera"));
    m_ptrDetector = dynamic_pointer_cast<AbstractBinpickingVisionDetector>(objectGroup.getObject("VisionDetector"));
    m_ptrPicker = dynamic_pointer_cast<AbstractBinpickingPicker>(objectGroup.getObject("Picker"));
    m_ptrPlacer = dynamic_pointer_cast<AbstractBinpickingPlacer>(objectGroup.getObject("Placer"));

    if (m_ptrRobot == nullptr) {
        COBOT_LOG.error() << "fail to init robot.";
        return false;
    }

    if (m_ptrKinematicSolver == nullptr) {
        COBOT_LOG.error() << "fail to init kinematic solver.";
        return false;
    }

    if (m_ptrMover == nullptr) {
        COBOT_LOG.error() << "fail to init robot mover.";
        return false;
    }

    if (m_ptrCameraMaster == nullptr) {
        COBOT_LOG.error() << "fail to init master camera.";
        return false;
    }

    if (m_ptrPicker == nullptr) {
        COBOT_LOG.error() << "fail to init picker.";
        return false;
    }

    if (m_ptrPlacer == nullptr) {
        COBOT_LOG.error() << "fail to init placer.";
        return false;
    }

    m_ptrMover->setKinematicSolver(m_ptrKinematicSolver);
    m_ptrMover->setRealTimeDriver(m_ptrRobot);
    m_ptrMover->start();

    auto _self = shared_from_this();

    m_ptrRobot->attach(std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(_self));
    m_ptrRobot->attach(std::dynamic_pointer_cast<ArmRobotRealTimeStatusObserver>(m_ptrViewer));

    m_ptrCameraMaster->attach(std::dynamic_pointer_cast<CameraStreamObserver>(_self));

    m_ptrPicker->setRobotDriver(m_ptrMover);

    return true;
}


void PhysicalDistributionController::clearAttachedObject() {
    detachSharedObject(m_ptrRobot);
    detachSharedObject(m_ptrCameraMaster);
    detachSharedObject(m_ptrManiputor);
}



#define ACTION_STEP(_what) if (m_loop) { _what; } else { break; }

void PhysicalDistributionController::mainLoop() {
    std::mutex loopmutex;
    std::unique_lock<std::mutex> uniqueLock(loopmutex);
    JsonServer::TaskInfo taskInfo;

    while (m_loop) {
        ACTION_STEP();
        if (!m_jsonServer->api_pickTask(taskInfo)) { // No task in server queue.
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // Notify Task, To WebSocket Interface.
        m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::Begin);

        // First, capture camera image.
        if (!_stepCaptureImage(uniqueLock)) break;

        // 调用视觉处理函数
        if (m_ptrDetector->processVisionImage(m_images)) {

            std::vector<BinObjGrabPose> results;
            ACTION_STEP(m_ptrDetector->getPickObjects(results));

            for (auto& obj : results) {
                // Pick, 这里是带有路径规划的抓取
                ACTION_STEP(m_ptrPicker->pickObject(obj));
                m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::PickFinish);

                // Place, 目的地，控制机器人移动到任务指定的目的地。然后放下物体。
                ACTION_STEP(m_ptrPlacer->placeObject());
                m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::DropFinish);
            }
            ACTION_STEP();
        }
        ACTION_STEP();
        m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::End);
    }
    COBOT_LOG.info() << "Main loop thread finished.";
}


void PhysicalDistributionController::onCameraStreamUpdate(const CameraFrame& cameraFrame, AbstractCamera* camera) {
    m_numImageCaptured++;
    m_imageUpdated = true;
    for (const auto& iter : cameraFrame.frames) {
        m_images.push_back({camera->getSerialNumber(), cameraFrame.capture_time, iter.data, iter.type});
    }
    m_cond.notify_all();
}

bool PhysicalDistributionController::_stepCaptureImage(std::unique_lock<std::mutex>& uniqueLock) {
    m_imageUpdated = false;
    m_images.clear();
    if (!m_ptrCameraMaster->capture(1000)) {
        COBOT_LOG.error() << "Fail to capture image, " << m_numImageCaptured;
        // TODO, here re-connect camera.
    }

    if (m_imageUpdated) {
    } else {
        m_cond.wait(uniqueLock);
    }
    _debugImages();
    return m_loop;
}

void PhysicalDistributionController::setupUi() {
    ui.setupUi(this);
    m_matViewer = new MatViewer(this);
    m_matViewer->getMatMerger().loadMatLayout(FileFinder::find("binpicking_image_layout.xml"));

    m_ptrViewer = std::make_shared<RobotStatusViewer>();
    m_ptrManiputor = std::make_shared<RobotManipulator>();

    ui.vBox->addWidget(m_matViewer);
    ui.hBox->addWidget(m_ptrViewer.get());
    ui.hBox->addWidget(m_ptrManiputor.get());
}

cv::Mat toColor(const cv::Mat& depth) {
    double vmin, vmax, alpha;
    cv::Mat color, gray;
    cv::minMaxLoc(depth, &vmin, &vmax);
    alpha = 255.0 / (vmax - vmin);
    depth.convertTo(gray, CV_8UC1, alpha, -vmin * alpha);
    cv::applyColorMap(gray, color, cv::COLORMAP_JET);
    return color;
}

cv::Mat toGray(const cv::Mat& ir) {
    double vmin, vmax, alpha;
    cv::Mat gray;
    cv::minMaxLoc(ir, &vmin, &vmax);
    alpha = 255.0 / (vmax - vmin);
    ir.convertTo(gray, CV_8UC1, alpha, -vmin * alpha);
    return gray;
}

void PhysicalDistributionController::_debugImages() {
    //m_matViewer->getMatMerger().updateMat("kin2color", m_images[0].image);

    m_matViewer->getMatMerger().updateMat("depth", toColor(m_images[3].image));
    m_matViewer->getMatMerger().updateMat("ir", m_images[4].image);
}


