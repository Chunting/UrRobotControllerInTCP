//
// Created by 潘绪洋 on 17-5-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//


#include <cobotsys.h>
#include <extra2.h>
#include <cobotsys_global_object_factory.h>
#include <cobotsys_abstract_camera.h>
#include <cobotsys_file_finder.h>
#include <cobotsys_abstract_controller.h>
#include <QApplication>
#include <QFileDialog>

int main(int argc, char** argv) {
    QApplication a(argc, argv);

    // 初始化cobotsys内部的变量
    cobotsys::init_library(argc, argv);

    // cobotsys的对象工厂
    cobotsys::GlobalObjectFactory globalObjectFactory;

    // 加载默认路径下的所有插件
    globalObjectFactory.loadLibrarys();

    auto pRobot = std::dynamic_pointer_cast<cobotsys::AbstractArmRobotRealTimeDriver>(
            globalObjectFactory.createObject("URRealTimeDriverFactory, Ver 1.0", "URRealTimeDriver"));
    auto pMover = std::dynamic_pointer_cast<cobotsys::AbstractArmRobotMoveDriver>(
            globalObjectFactory.createObject("UrMoverFactory, Ver 1.0", "UrMover"));
    auto pSolver = std::dynamic_pointer_cast<cobotsys::AbstractKinematicSolver>(
            globalObjectFactory.createObject("KinematicSolverFactory, Ver 1.0", "KinematicSolver"));
    auto pLogger = std::dynamic_pointer_cast<cobotsys::AbstractWidget>(
            globalObjectFactory.createObject("SimpleUiFactory, Ver 1.0", "BasicLoggerWidget"));

    if (pRobot && pMover && pSolver) {
        if (!pRobot->setup("CONFIG/UrRobotConfig/ur_sim_config.json")) {
            COBOT_LOG.warning() << "Fail to setup robot.";
            return 1;
        }
        if (!pMover->setup("")) {
            COBOT_LOG.warning() << "Fail to setup mover.";
            return 1;
        }
        if (!pSolver->setup("CONFIG/UrRobotConfig/ur5_180_config.json")) {
            COBOT_LOG.warning() << "Fail to setup solver.";
            return 3;
        }

        pLogger->setup("");

        pMover->setKinematicSolver(pSolver);
        pMover->setRealTimeDriver(pRobot);
        pMover->start();

        pRobot->start();

        bool testThread = true;
        std::thread moverTest([&]() {
            while (testThread) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                if (pRobot->isStarted()) {
                    pMover->move(pMover->generateMoveId(), {0.1, 0.1, 0.3}, {0, 0, 0});
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    pMover->move(pMover->generateMoveId(), {0.2, 0.1, 0.5}, {0, 0, 0});
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    COBOT_LOG.error("sss") << "fail to start robot";

                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    pRobot->start();

                }
            }
        });

        pLogger->show();
        auto r = a.exec();
        testThread = false;
        if (moverTest.joinable()) moverTest.join();
        return r;
    }
    return 0;
}