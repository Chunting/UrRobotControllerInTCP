//
// Created by lhc on 17-4-10.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_FORCE_SENSOR_H
#define PROJECT_COBOTSYS_ABSTRACT_FORCE_SENSOR_H

#include "cobotsys.h"
#include "cobotsys_abstract_object.h"
#include <vector>
#include <chrono>
#include "cobotsys_data_types.h"


namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 */

/**
 * @defgroup forcesensor
 * @brief 力传感器API接口
 * @{
 */

class ForceSensorStreamObserver {
public:
	ForceSensorStreamObserver();
    virtual ~ForceSensorStreamObserver();
	virtual void onForceSensorConnect() = 0;
	virtual void onForceSensorDisconnect() = 0;
    virtual void onForceSensorDataStreamUpdate(const std::shared_ptr<forcesensor::Wrench>& ptrWrench) = 0;
};

class AbstractForceSensor : public AbstractObject {
public:
	AbstractForceSensor();
    virtual ~AbstractForceSensor();

    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual void attach(const shared_ptr<ForceSensorStreamObserver>& observer) = 0;

};

/**
 * @}
 * @}
 */
}


#endif //PROJECT_COBOTSYS_ABSTRACT_FORCE_SENSOR_H
