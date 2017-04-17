//
// Created by longhuicai on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_OPTOFORCESENSOR_H
#define PROJECT_FORCECONTROLLER_WRAPPER_H

#include <mutex>
#include <thread>
#include <cobotsys_abstract_object.h>
#include <QObject>
#include <QString>
#include "ForceController.h"
using namespace cobotsys;

class ForceController_wrapper : public  AbstractObject,public ForceControllerModelClass {
public:
	ForceController_wrapper();
	virtual ~ForceController_wrapper();
	virtual bool setup(const QString& configFilePath);
};


#endif //PROJECT_FORCECONTROLLER_WRAPPER_H
