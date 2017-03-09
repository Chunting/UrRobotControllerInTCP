//
// Created by 潘绪洋 on 17-3-8.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_camera.h"

/// @note CameraInformation
namespace cobotsys {
class CameraInformation::CameraInformationImpl {
public:
    struct FrameInfo {
        int width;
        int height;
        CameraFrameType type;
    };


    std::string manufacturer;
    std::string fullDescription;
    std::string serialNumber;
    std::vector<FrameInfo> frameInfos;

    void parserInfo(const std::string& info){
    }

    FrameInfo getFrameInfo(int frameId) const{
        if (frameId >= 0 && frameId < (int) frameInfos.size()) {
            return frameInfos[frameId];
        }
        return {-1, -1, CameraFrameType::Color};
    }
};


CameraInformation::CameraInformation()
        : m_impl(new CameraInformationImpl){
}

CameraInformation::CameraInformation(const std::string& info)
        : m_impl(new CameraInformationImpl){
    m_impl->parserInfo(info);
}

CameraInformation::CameraInformation(const CameraInformation& other)
        : m_impl(other.m_impl){
}

std::string CameraInformation::getManufacturer() const{
    return m_impl->manufacturer;
}

std::string CameraInformation::getFullDescription() const{
    return m_impl->fullDescription;
}

std::string CameraInformation::getSerialNumber() const{
    return m_impl->serialNumber;
}

int CameraInformation::getFrameWidth(int frameId) const{
    return m_impl->getFrameInfo(frameId).width;
}

int CameraInformation::getFrameHeight(int frameId) const{
    return m_impl->getFrameInfo(frameId).height;
}

CameraFrameType CameraInformation::getFrameType(int frameId) const{
    return m_impl->getFrameInfo(frameId).type;
}

int CameraInformation::getFrameCount() const{
    return (int) (m_impl->frameInfos.size());
}
}


namespace cobotsys {
AbstractCamera::AbstractCamera(){
}

AbstractCamera::~AbstractCamera(){
}
}


namespace cobotsys {
CameraStreamObserver::CameraStreamObserver(){
}

CameraStreamObserver::~CameraStreamObserver(){
}
}
