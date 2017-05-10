//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCE_POLISHING_CONTROLLER_H
#define PROJECT_FORCE_POLISHING_CONTROLLER_H


#include "cobotsys_abstract_controller.h"
#include "polishingtask.h"
#include <QObject>
#include <cobotsys_global_object_factory.h>
#include <cobotsys_file_finder.h>

#include <cobotsys_abstract_arm_robot_move_driver.h>
#include <cobotsys_abstract_arm_robot_realtime_driver.h>
#include <cobotsys_abstract_kinematic_solver.h>
#include <cobotsys_abstract_force_sensor.h>
using namespace cobotsys;
class ForcePolishingController :
        public cobotsys::AbstractController{
public:

	ForcePolishingController();
    virtual ~ForcePolishingController();

public:
    virtual bool start();
    virtual void pause();
    virtual void stop();
    virtual bool setup(const QString& configFilePath);
protected:
    void cartesian_path_generator();
    KDL::Frame getPolisherFrame(int id);
    void defineInterimPath();
    bool checkPathContinuity(int path_index);
    std::vector<KDL::Frame> planInterimPath(int path_index);
protected:
	std::string m_model_path;
	std::string m_ptd_path;
    PolishingTask m_polishingTask;

    std::vector<std::vector<KDL::Frame> > cartesian_path_;
    std::vector<std::vector<KDL::Frame> > interim_path_;
};

#endif //PROJECT_FORCE_POLISHING_CONTROLLER_H
