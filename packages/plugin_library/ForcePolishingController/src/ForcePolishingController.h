//
// Created by 杨帆 on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_FORCE_POLISHING_CONTROLLER_H
#define PROJECT_FORCE_POLISHING_CONTROLLER_H


#include "cobotsys_abstract_controller.h"
#include <QObject>

using namespace cobotsys;
class ForcePolishingController : public cobotsys::AbstractController {
public:
	ForcePolishingController();
    virtual ~ForcePolishingController();

public:
    virtual bool start();
    virtual void pause();
    virtual void stop();
    virtual bool setup(const QString& configFilePath);
protected:
	std::string m_model_path;
	std::string m_ptd_path;
};

#endif //PROJECT_FORCE_POLISHING_CONTROLLER_H
