//
// Created by 于天水 on 17-4-12.
//

#ifndef COBOTSYS_FOTONICCAMERA_H
#define COBOTSYS_FOTONICCAMERA_H

#include <cobotsys_abstract_object_factory.h>
#include <cobotsys_abstract_camera.h>
#include <OpenNI.h>
#include <mutex>

class FotonicCamera  : public cobotsys::AbstractCamera {
public:
    FotonicCamera();
    virtual ~FotonicCamera();

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

    openni::Device m_device;
    openni::VideoStream m_depthStream;
    openni::VideoStream m_colorStream;
    openni::Array<openni::DeviceInfo> m_deviceList;

    std::vector<std::shared_ptr<cobotsys::CameraStreamObserver> > m_observers;
};



#endif //COBOTSYS_FOTONICCAMERA_H
