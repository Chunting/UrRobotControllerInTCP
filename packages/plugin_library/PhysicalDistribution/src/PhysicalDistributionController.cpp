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
    m_taskEmptyPrint = true;
    m_taskEmpty = true;
    m_taskEmptyDebugAction = false;
    m_cancelCurrentTask = false;
    m_jsonServer = new JsonServer(this);
    connect(m_jsonServer, &JsonServer::reqStart, this, &PhysicalDistributionController::start);
    connect(m_jsonServer, &JsonServer::reqStop, this, &PhysicalDistributionController::stop);

    setupUi();
}

PhysicalDistributionController::~PhysicalDistributionController() {
    // 使用shared_ptr管理的对象，在释放的时候，需要主动的从Qt的管理方法里删除。不
    // 然，会造成两次对象释放，让程序崩溃。
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
        } else {
            COBOT_LOG.error() << "fail to init all objects";
            return false;
        }
    }
    COBOT_LOG.error() << "fail to load json config.";
    return false;
}

bool PhysicalDistributionController::start() {
    bool success = false;
    if (m_ptrRobot) {
        success = m_ptrRobot->start();
        if (!success) COBOT_LOG.error() << "Fail To Start Robot!";

        if (success && !m_ptrCameraMaster->isOpened()) {
            success = m_ptrCameraMaster->open();
            if (!success) COBOT_LOG.error() << "Fail To Start Camera!";
            if (!m_mainTaskThread.joinable()) {
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
    std::lock_guard<std::mutex> lockctx(m_mutex);
    m_loop = false;
    if (m_ptrMover) {
        m_ptrMover->clearAll();
    }

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

    cv::destroyAllWindows();
}

void PhysicalDistributionController::onArmRobotConnect() {
    std::lock_guard<std::mutex> lockctx(m_mutex);
    m_robotConnected = true;
}

void PhysicalDistributionController::onArmRobotDisconnect() {
    std::lock_guard<std::mutex> lockctx(m_mutex);
    m_robotConnected = false;
    COBOT_LOG.notice() << "Robot Driver Disconnected.";
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

    auto jfilter = dynamic_pointer_cast<ArmRobotJointTargetFilter>(objectGroup.getObject("JointFilter"));

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
    if (jfilter) {
        m_ptrRobot->setTargetJointFilter(jfilter);
    }


    m_ptrCameraMaster->attach(std::dynamic_pointer_cast<CameraStreamObserver>(_self));

    m_ptrPicker->setRobotDriver(m_ptrMover);
    m_ptrPicker->setDigitIoDriver(m_ptrRobot->getDigitIoDriver());

    m_ptrManiputor->setRobotMover(m_ptrRobot, m_ptrMover);

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

    m_taskEmptyPrint = true;
    bool visionSuccess = true;

    COBOT_LOG.notice() << "PhysicalDistributionController::mainLoop Started.";
    while (m_loop) {
        ACTION_STEP();
        if (visionSuccess) { // 只有前一个任务处理成功，才可以处理下一个任务。
            if (m_jsonServer->api_pickTask(taskInfo)) {
                COBOT_LOG.notice() << "Task: " << taskInfo.objFrom << " -> " << taskInfo.objTo;
            } else { // No task in server queue.
                m_taskEmpty = true;
                if (m_taskEmptyPrint) {
                    m_taskEmptyPrint = false;
                    COBOT_LOG.notice() << "No Task In Queue.";
                }
                if (m_taskEmptyDebugAction) {
                    _doTestActions();
                    m_taskEmptyDebugAction = false;
                }

                // Try to preview image;
                _stepCaptureImage(uniqueLock);

                // Sleep, and Continue
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            m_taskEmptyPrint = true;

            // Notify Task, To WebSocket Interface.
            m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::Begin);
        } else if (m_cancelCurrentTask) { // 终止当前任务
            m_cancelCurrentTask = false;
            m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::PickPlaceFailure);
            m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::End);
            visionSuccess = true;
            continue;
        }


        // First, capture camera image.
        if (!_stepCaptureImage(uniqueLock)) break;

        // 调用视觉处理函数
        visionSuccess = m_ptrDetector->processVisionImage(m_images);
        if (visionSuccess) {
            std::vector<BinObjGrabPose> results;
            ACTION_STEP(m_ptrDetector->getPickObjects(results));

            for (auto& obj : results) {
                // Pick, 这里是带有路径规划的抓取
                if (m_ptrPicker->pickObject(obj)) { // 抓取成功，则发送信息
                    m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::PickFinish);
                } else {
                    // TODO 这里的流程控制需要做更多的工作。与外部系统集成
                    COBOT_LOG.error() << "Picker fail, lookup log for more information.";
                }
                ACTION_STEP();

                // Place, 目的地，控制机器人移动到任务指定的目的地。然后放下物体。
                ACTION_STEP(m_ptrPlacer->placeObject());
                m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::DropFinish);
            }
            ACTION_STEP();
            m_jsonServer->api_setupTaskStage(taskInfo, JsonServer::TaskStage::End);
        } else {
            COBOT_LOG.warning() << "Vision Detector Fail to Compute Target Grab Pose. Retry...";
        }
        ACTION_STEP();
    }
    COBOT_LOG.info() << "Main loop thread finished.";
}


void PhysicalDistributionController::_doTestActions() {
    if (m_doTestPicker) {
        m_doTestPicker = false;
        if (m_ptrPicker) {
            m_ptrPicker->pickObject(m_testPickerPose);
        }
    } else if (m_doTestPlacer) {
        m_doTestPlacer = false;
    }
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
    static int num_print = 0;
    m_imageUpdated = false;
    m_images.clear();
    if (!m_ptrCameraMaster->capture(1000)) {
        if (num_print < 1) {
            COBOT_LOG.error() << "Fail to capture image, Num Captured: " << m_numImageCaptured;
            num_print++;
        }
        m_imageUpdated = true; // For Debug Only


        // TODO, here re-connect camera.
    } else {
        num_print = 0;
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

    connect(ui.btnTESTPicker, &QPushButton::released, this, &PhysicalDistributionController::onButtonTestPicker);
    connect(ui.btnTESTPlacer, &QPushButton::released, this, &PhysicalDistributionController::onButtonTestPlacer);
    connect(this, &PhysicalDistributionController::debugImageUpdated,
            this, &PhysicalDistributionController::rendererDebugImage);

    connect(ui.btnTESTTask_0_1, &QPushButton::released, this, &PhysicalDistributionController::onButtonTestTask0_1);
    connect(ui.btnTESTTask_0_2, &QPushButton::released, [=]() { m_jsonServer->api_debugTaskOnce("0", "2"); });
    connect(ui.btnTESTTask_0_3, &QPushButton::released, [=]() { m_jsonServer->api_debugTaskOnce("0", "3"); });
    connect(ui.btnTESTTask_0_4, &QPushButton::released, [=]() { m_jsonServer->api_debugTaskOnce("0", "4"); });

    connect(ui.btnTaskCancel, &QPushButton::released, this, &PhysicalDistributionController::onButtonCancelCurrentTask);
}

void PhysicalDistributionController::onButtonTestPicker() {
    /**
     * 这里，全部使用bool标志是为了让移动操作在mainLoop()里进行运动控制
     * 这里仅仅只是赋值。
     */
    if (m_taskEmpty) {
        m_doTestPicker = true;
        m_testPickerPose.target_info = "TEST";
        m_testPickerPose.position = {100 / 1000.0, -100 / 1000.0, 500 / 1000.0};
        m_testPickerPose.rotation = {0, 0, 0};
        m_taskEmptyDebugAction = true;
    }
}

void PhysicalDistributionController::onButtonTestPlacer() {
    if (m_taskEmpty) {
        m_doTestPlacer = true;
        m_taskEmptyDebugAction = true;
    }
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

    if (m_images.size() >= 5) {
        m_matViewer->getMatMerger().updateMat("depth", toColor(m_images[3].image));
        m_matViewer->getMatMerger().updateMat("ir", m_images[4].image);

        m_ptrDetector->debugMat("sss", m_images[4].image);
    } else {
        if (m_images.size()) {
            m_ptrDetector->debugMat("Sample", m_images[0].image);
        }
    }
    Q_EMIT debugImageUpdated();
}

void PhysicalDistributionController::rendererDebugImage() {
    m_ptrDetector->debugRenderer();
}

void PhysicalDistributionController::onButtonTestTask0_1() {
    m_jsonServer->api_debugTaskOnce("0", "1");
}

void PhysicalDistributionController::onButtonCancelCurrentTask() {
    m_cancelCurrentTask = true;
}



