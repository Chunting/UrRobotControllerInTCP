//
// Created by lhc on 17-4-10.
//

#include "OptoForceSensor.h"

OptoForceSensor::OptoForceSensor() {

}

OptoForceSensor::~OptoForceSensor() {

}

bool OptoForceSensor::setup(const QString &configFilePath) {
	return true;
}

bool OptoForceSensor::open(int deviceId = 0) {
	return true;
}

void OptoForceSensor::close() {
}

void OptoForceSensor::attach(const shared_ptr<ForceSensorStreamObserver>& observer) {
}