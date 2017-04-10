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
    virtual void onForceSensorDataStreamUpdate(const forcesensor::Wrench& wrench) = 0;
};

class AbstractForceSensor : public AbstractObject {
public:
	AbstractForceSensor();
    virtual ~AbstractForceSensor();

    virtual bool open(int deviceId = 0) = 0;
    virtual void close() = 0;
    virtual void attach(const shared_ptr<ForceSensorStreamObserver>& observer) = 0;

};

/**
 * @}
 * @}
 */
}


#endif //PROJECT_COBOTSYS_ABSTRACT_FORCE_SENSOR_H
