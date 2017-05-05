//
// Created by 于天水 on 17-4-12.
//

#include "FotonicCamera.h"
#include "cobotsys_logger.h"

using namespace cobotsys;
using namespace openni;

FotonicCamera::FotonicCamera() {
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        COBOT_LOG.info() << "Initialize failed" << OpenNI::getExtendedError() ;
        return ;
    }

    openni::OpenNI::enumerateDevices(&m_deviceList);
}

FotonicCamera::~FotonicCamera() {
    OpenNI::shutdown();
}

bool FotonicCamera::isOpened() const {
    return m_device.isValid();
}

bool FotonicCamera::open(int deviceId) {
    if (m_deviceList.getSize() <= deviceId) {
        COBOT_LOG.info() << "The Device " <<  deviceId << " is not find...";
        return false;
    }
    COBOT_LOG.info() << "The Device size:" << m_deviceList.getSize() ;
    std::lock_guard<std::mutex> lock(m_ioMutex);
    if (m_device.isValid())
        return true;

    COBOT_LOG.info() << "The Device uri:" << m_deviceList[deviceId].getUri() ;
    Status rc = m_device.open(m_deviceList[deviceId].getUri());

    if (rc != STATUS_OK)
    {
        COBOT_LOG.info() << "Couldn't open device " << OpenNI::getExtendedError() ;
        return false;
    }

    if (m_device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        rc = m_colorStream.create(m_device, SENSOR_COLOR);
        if (rc == STATUS_OK)
        {
            rc = m_colorStream.start();
            if (rc != STATUS_OK)
            {
                COBOT_LOG.info() << "Couldn't start the color stream " << OpenNI::getExtendedError() ;
            }
        }
        else
        {
            COBOT_LOG.info() << "Couldn't create color stream " << OpenNI::getExtendedError() ;
        }
    }

    if (m_device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = m_depthStream.create(m_device, SENSOR_DEPTH);
        if (rc == STATUS_OK)
        {
            rc = m_depthStream.start();
            if (rc != STATUS_OK)
            {
                COBOT_LOG.info() << "Couldn't start the depth stream " << OpenNI::getExtendedError() ;
            }
        }
        else
        {
            COBOT_LOG.info() << "Couldn't create depth stream " << OpenNI::getExtendedError() ;
        }
    }

//    if (m_device.getSensorInfo(SENSOR_IR) != NULL)
//    {
//        rc = m_depthStream.create(m_device, SENSOR_IR);
//        if (rc == STATUS_OK)
//        {
//            rc = m_depthStream.start();
//            if (rc != STATUS_OK)
//            {
//                COBOT_LOG.info() << "Couldn't start the ir stream " << OpenNI::getExtendedError() ;
//            }
//        }
//        else
//        {
//            COBOT_LOG.info() << "Couldn't create ir stream " << OpenNI::getExtendedError() ;
//        }
//    }


    //m_device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    return true;
}

void FotonicCamera::close() {
    std::lock_guard<std::mutex> lock(m_ioMutex);

    m_depthStream.stop();
    m_colorStream.stop();
    m_depthStream.destroy();
    m_colorStream.destroy();
    m_device.close();
}

void FotonicCamera::attach(const std::shared_ptr<cobotsys::CameraStreamObserver>& observer) {
    if (observer && find(m_observers.begin(), m_observers.end(), observer) == m_observers.end())
        m_observers.push_back(observer);
}

bool FotonicCamera::capture(int waitMs) {
    std::lock_guard<std::mutex> lock(m_ioMutex);

    if (!m_device.isValid())
        return false;

    VideoFrameRef colorFrame;
    VideoFrameRef depthFrame;

    VideoStream* streams[] = {&m_depthStream, &m_colorStream};

    while (!depthFrame.isValid() || !colorFrame.isValid())
    {
        int readyStream = -1;
        Status rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, waitMs);
        if (rc != STATUS_OK)
        {
            COBOT_LOG.info() << "Wait failed! (timeout is " << waitMs << " ms) " << OpenNI::getExtendedError() ;
            return false;
        }

        switch (readyStream)
        {
            case 0:
                // Depth
                m_depthStream.readFrame(&depthFrame);
//                COBOT_LOG.info() << "m_depth.readFrame:" << depthFrame.getData() ;
                break;
            case 1:
                // Color
                m_colorStream.readFrame(&colorFrame);
//                COBOT_LOG.info() << "m_color.readFrame:" << colorFrame.getData() ;
                break;
            default:
                COBOT_LOG.info() << "Unxpected stream..." ;
        }
    }

    cv::Mat raw_color(colorFrame.getHeight(), colorFrame.getWidth(), CV_8UC3, (uint8_t*)colorFrame.getData());
    cv::Mat raw_depth(depthFrame.getHeight(), depthFrame.getWidth(), CV_16UC1, (DepthPixel*)depthFrame.getData());

    int pColorX, pColorY;
    CoordinateConverter::convertDepthToColor(m_depthStream, m_colorStream, 10, 10, raw_depth.at<uint16_t>(10, 10), &pColorX, &pColorY);
    COBOT_LOG.info() << "convertDepthToColor---x:" << pColorX << "     y:" << pColorY;

    float pWorldX, pWorldY, pWorldZ;
    CoordinateConverter::convertDepthToWorld(m_depthStream, 10, 10, raw_color.at<uint16_t>(10, 10), &pWorldX, &pWorldY, &pWorldZ);
    COBOT_LOG.info() << "convertDepthToWorld---x:" << pWorldX << "     y:" << pWorldY << "     z:" << pWorldZ;
    COBOT_LOG.info() << "depthFrame---Height:" << depthFrame.getHeight() << "   m_depthframe---Width:" << depthFrame.getWidth() << "   SIZE:" << depthFrame.getDataSize() ;


    cobotsys::ImageFrame c_color = {cobotsys::ImageType::Color, raw_color};
    cobotsys::ImageFrame c_depth = {cobotsys::ImageType::Depth, raw_depth};

    cobotsys::CameraFrame streamFrames;
    streamFrames.capture_time = std::chrono::high_resolution_clock::from_time_t(depthFrame.getTimestamp());
    streamFrames.frames.push_back(c_color);
    streamFrames.frames.push_back(c_depth);

    notify(streamFrames);
}

std::string FotonicCamera::getManufacturer() const {
    return m_device.getDeviceInfo().getVendor();
}

std::string FotonicCamera::getFullDescription() const {
    return m_device.getDeviceInfo().getName();
}

std::string FotonicCamera::getSerialNumber() const {
    return m_device.getDeviceInfo().getUri();
}

int FotonicCamera::getImageWidth(int imageIdx) const {
    switch (imageIdx) {
        case 0:
            if (m_colorStream.isValid())
                return m_colorStream.getVideoMode().getResolutionX();
            else
                return 640;
        case 1:
            if (m_depthStream.isValid())
                return m_depthStream.getVideoMode().getResolutionX();
            else
                return 640;
        default:
            return 640;
    }
}

int FotonicCamera::getImageHeight(int imageIdx) const {
    switch (imageIdx) {
        case 0:
            if (m_colorStream.isValid())
                return m_colorStream.getVideoMode().getResolutionY();
            else
                return 640;
        case 1:
            if (m_depthStream.isValid())
                return m_depthStream.getVideoMode().getResolutionY();
            else
                return 640;
        default:
            return 640;
    }
}

ImageType FotonicCamera::getImageType(int imageIdx) const {
    switch (imageIdx) {
        case 0:
            return ImageType::Color;
        case 1:
            return ImageType::Depth;
        default:
            return ImageType::Color;
    }
}

int FotonicCamera::getImageCount() const {
    return 2;
}

void FotonicCamera::notify(const cobotsys::CameraFrame& cameraFrame) {
    for (auto& observer : m_observers) {
        observer->onCameraStreamUpdate(cameraFrame, nullptr);
    }
}