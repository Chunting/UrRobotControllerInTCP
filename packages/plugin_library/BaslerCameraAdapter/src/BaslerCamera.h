//
// Created by 于天水 on 17-4-12.
//

#ifndef COBOTSYS_BASLERCAMERA_H
#define COBOTSYS_BASLERCAMERA_H

#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_camera.h>
#include <mutex>

#include <pylon/PylonIncludes.h>
#define USE_GIGE 1
#if defined(USE_1394)
#include <pylon/1394/Basler1394InstantCamera.h>
typedef Pylon::CBasler1394InstantCamera Camera_t;
typedef Pylon::CBasler1394ImageEventHandler ImageEventHandler_t;
typedef Pylon::CBasler1394GrabResultPtr GrabResultPtr_t;
using namespace Basler_IIDC1394CameraParams;
#elif defined (USE_GIGE)
#include <pylon/gige/BaslerGigEInstantCamera.h>
typedef Pylon::CBaslerGigEInstantCamera Camera_t;
typedef Pylon::CBaslerGigEImageEventHandler ImageEventHandler_t;
typedef Pylon::CBaslerGigEGrabResultPtr GrabResultPtr_t;
using namespace Basler_GigECameraParams;
#elif defined(USE_USB)
#include <pylon/usb/BaslerUsbInstantCamera.h>
typedef Pylon::CBaslerUsbInstantCamera Camera_t;
typedef Pylon::CBaslerUsbImageEventHandler ImageEventHandler_t;
typedef Pylon::CBaslerUsbGrabResultPtr GrabResultPtr_t;
using namespace Basler_UsbCameraParams;
#else
#error Camera type is not specified. For example, define USE_GIGE for using GigE cameras.
#endif

class BaslerCamera : public cobotsys::AbstractCamera {
public:
    BaslerCamera();
    virtual ~BaslerCamera();

    virtual bool isOpened() const;
    virtual bool open(int deviceId = 0);
    virtual void close(); ///  @note 最好不要在回调函数里调用close函数。
    virtual void attach(const std::shared_ptr<cobotsys::CameraStreamObserver>& observer);
    virtual bool capture(int waitMs); /// @note 控制相机进行一次图像捕获

    virtual bool setup(const QString& configFilePath) { return true; }

    virtual std::string getManufacturer() const;
    virtual std::string getFullDescription() const;
    virtual std::string getSerialNumber() const;

    virtual int getImageWidth(int imageIdx = 0) const;
    virtual int getImageHeight(int imageIdx = 0) const;
    virtual cobotsys::ImageType getImageType(int imageIdx = 0) const;
    virtual int getImageCount() const;
protected:
    void notify(const cobotsys::CameraFrame& cameraFrame);
protected:
    std::mutex m_ioMutex;

    Pylon::PylonAutoInitTerm autoInitTerm;
    std::shared_ptr<Pylon::CImageFormatConverter> m_formatConverter;
    Pylon::DeviceInfoList_t m_devices;
    std::shared_ptr<Camera_t> m_camera;

    std::vector<std::shared_ptr<cobotsys::CameraStreamObserver> > m_observers;
};


#endif //COBOTSYS_BASLERCAMERA_H
