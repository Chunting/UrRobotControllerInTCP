//
// Created by 潘绪洋 on 17-4-24.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_PHYSICALDISTRIBUTIONCONTROLLER_H
#define COBOTSYS_PHYSICALDISTRIBUTIONCONTROLLER_H

#include <cobotsys_abstract_controller.h>
#include <cobotsys_global_object_factory.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_kinematic_solver.h>
#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <mutex>
#include <cobotsys_abstract_camera.h>
#include <cobotsys_abstract_binpicking_vision_detector.h>
#include <cobotsys_abstract_binpicking_picker.h>
#include <cobotsys_abstract_binpicking_placer.h>
#include <condition_variable>

#include "JsonServer.h"
#include "MatViewer.h"
#include "RobotStatusViewer.h"
#include <thread>
#include <MatMerger.h>
#include <QVBoxLayout>
#include "ui_PhysicalDistributionController.h"
#include "RobotManipulator.h"

using namespace cobotsys;
using namespace cobotsys::binpicking;

class PhysicalDistributionController : public AbstractControllerWidget,
                                       public ArmRobotRealTimeStatusObserver,
                                       public CameraStreamObserver {
Q_OBJECT
public:
    PhysicalDistributionController();
    virtual ~PhysicalDistributionController();

    virtual bool setup(const QString& configFilePath);

    virtual bool start();
    virtual void pause();
    virtual void stop();

    virtual void clearAttachedObject();

Q_SIGNALS:
    void debugImageUpdated();

protected:
    void mainLoop();
    void setupUi();

    void onButtonTestPicker();
    void onButtonTestPlacer();

    void rendererDebugImage();

protected:
    virtual void onArmRobotConnect();
    virtual void onArmRobotDisconnect();
    virtual void onArmRobotStatusUpdate(const ArmRobotStatusPtr& ptrRobotStatus);

    virtual void onCameraStreamUpdate(const CameraFrame& cameraFrame, AbstractCamera* camera);

protected:
    bool _setupInternalObjects(ObjectGroup& objectGroup);
    void _debugImages();


    bool _stepCaptureImage(std::unique_lock<std::mutex>& uniqueLock);
protected:
    std::shared_ptr<AbstractArmRobotRealTimeDriver> m_ptrRobot;
    std::shared_ptr<AbstractKinematicSolver> m_ptrKinematicSolver;
    std::shared_ptr<AbstractArmRobotMoveDriver> m_ptrMover;
    std::shared_ptr<AbstractCamera> m_ptrCameraMaster;
    std::shared_ptr<AbstractBinpickingVisionDetector> m_ptrDetector;
    std::shared_ptr<AbstractBinpickingPicker> m_ptrPicker;
    std::shared_ptr<AbstractBinpickingPlacer> m_ptrPlacer;

    std::mutex m_mutex;
    bool m_robotConnected;

    JsonServer* m_jsonServer;

    std::thread m_mainTaskThread;
    bool m_loop;

    std::condition_variable m_cond;
    std::vector<VisionInputImage> m_images;
    bool m_imageUpdated;

    int m_numImageCaptured;

    MatViewer* m_matViewer;
    std::shared_ptr<RobotStatusViewer> m_ptrViewer;
    std::shared_ptr<RobotManipulator> m_ptrManiputor;

    template<class T>
    void removeSharedWidget(T& t) {
        if (t) {
            layout()->removeWidget(t.get());
        }
    }

    bool m_taskEmptyPrint;
    bool m_taskEmpty;

    bool m_taskEmptyDebugAction;
    bool m_doTestPicker;
    bool m_doTestPlacer;
    BinObjGrabPose m_testPickerPose;
    void _doTestActions();


    Ui::PhysicalDistributionController ui;
};


#endif //COBOTSYS_PHYSICALDISTRIBUTIONCONTROLLER_H
