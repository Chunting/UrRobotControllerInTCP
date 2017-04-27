//
// Created by 于天水 on 17-4-12.
//

#include "BaslerCamera.h"
#include <algorithm>
#include <chrono>
#include "cobotsys_logger.h"

using namespace Pylon;

BaslerCamera::BaslerCamera() {
    CTlFactory::GetInstance().EnumerateDevices(m_devices);
    m_formatConverter = std::make_shared<Pylon::CImageFormatConverter>();
    m_formatConverter->OutputPixelFormat = PixelType_BGR8packed;
}

BaslerCamera::~BaslerCamera() {
    close();
}

bool BaslerCamera::isOpened() const {
    if (m_camera && m_camera->IsOpen())
        return true;
    else
        return false;
}

bool BaslerCamera::open(int deviceId) {
    if (m_devices.size() <= deviceId) {
        COBOT_LOG.info() << "The Device " <<  deviceId << " is not find...";
        return false;
    }

    std::lock_guard<std::mutex> lock(m_ioMutex);

    if (m_camera && m_camera->IsOpen())
        return true;

    m_camera = std::make_shared<Camera_t>(CTlFactory::GetInstance().CreateDevice(m_devices[deviceId]));
    if (!m_camera) {
        COBOT_LOG.info() << "Create Camera(" << deviceId << ") faild, trying to re-open . . ." ;
        return false;
    }
    COBOT_LOG.info() << "Using device " << m_camera->GetDeviceInfo().GetModelName() ;

    m_camera->MaxNumBuffer = 1;
    m_camera->RegisterConfiguration(new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
    // Open the camera for setting parameters.
    m_camera->Open();
    // Start the grabbing of c_countOfImagesToGrab images.
    m_camera->StartGrabbing();

    return true;
}

void BaslerCamera::close() {
    std::lock_guard<std::mutex> lock(m_ioMutex);
    if (!m_camera)
        return;

    if (m_camera->IsGrabbing())
        m_camera->StopGrabbing();

    if (m_camera->IsOpen())
        m_camera->Close();

    m_camera.reset();
}

void BaslerCamera::attach(const std::shared_ptr<cobotsys::CameraStreamObserver>& observer) {
    if (observer && find(m_observers.begin(), m_observers.end(), observer) == m_observers.end())
        m_observers.push_back(observer);
}

bool BaslerCamera::capture(int waitMs) {
    std::lock_guard<std::mutex> lock(m_ioMutex);

    if (!m_camera || !m_camera->IsGrabbing())
        return false;

    // This smart pointer will receive the grab result data.
    CGrabResultPtr ptrGrabResult;

    // Execute the software trigger. Wait up to waitMs ms for the camera to be ready for trigger.
    if (m_camera->WaitForFrameTriggerReady(waitMs, TimeoutHandling_ThrowException))
    {
        m_camera->ExecuteSoftwareTrigger();
    }

    std::chrono::high_resolution_clock::time_point timeAfterWait =
            std::chrono::high_resolution_clock::now();
    // Retrieve grab results and notify the camera event and image event handlers.
    m_camera->RetrieveResult(waitMs, ptrGrabResult, TimeoutHandling_ThrowException);

    if (!ptrGrabResult.IsValid())
    {
        COBOT_LOG.info() << "camera grab RetrieveResult failed";
        return false;
    }

    CPylonImage pylonImage;
    m_formatConverter->Convert(pylonImage, ptrGrabResult);
    //COBOT_LOG.info() << "camera ptrGrabResult->GetHeight:" << ptrGrabResult->GetHeight() << "      ptrGrabResult->GetWidth:" << ptrGrabResult->GetWidth();
    cv::Mat raw_color = cv::Mat(ptrGrabResult->GetHeight(),
                                  ptrGrabResult->GetWidth(),
                                  CV_8UC3,
                                  (uint8_t *)pylonImage.GetBuffer());
    cobotsys::ImageFrame c_color = {cobotsys::ImageType::Color, raw_color};

    cobotsys::CameraFrame streamFrames;
    streamFrames.capture_time = timeAfterWait;
    streamFrames.frames.push_back(c_color);

    notify(streamFrames);
    return true;
}

std::string BaslerCamera::getManufacturer() const {
    return "";
}

std::string BaslerCamera::getFullDescription() const {
    return (m_camera ? m_camera->GetDeviceInfo().GetFullName().c_str() : "");
}

std::string BaslerCamera::getSerialNumber() const {
    return (m_camera ? m_camera->GetDeviceInfo().GetSerialNumber().c_str() : "");
}

int BaslerCamera::getImageWidth(int imageIdx) const {
    return 2590;
}

int BaslerCamera::getImageHeight(int imageIdx ) const {
    return 1942;
}

cobotsys::ImageType BaslerCamera::getImageType(int imageIdx) const {
    return cobotsys::ImageType::Color;
}

int BaslerCamera::getImageCount() const {
    return 1;
}

void BaslerCamera::notify(const cobotsys::CameraFrame& cameraFrame) {
    for (auto& observer : m_observers) {
        observer->onCameraStreamUpdate(cameraFrame);
    }
}