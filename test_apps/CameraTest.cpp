//
// Created by 于天水 on 17-4-13.
//

#include <cobotsys_abstract_camera.h>
#include <cobotsys_global_object_factory.h>
#include "highgui.h"
#include <QApplication>

using namespace cobotsys;

class FotonicObserver : public cobotsys::CameraStreamObserver {
    virtual void onCameraStreamUpdate(const CameraFrame& cameraFrame) {
        if (cameraFrame.frames.size() == 0)
            std::cout << "cameraFrame.frames.size()  is 0" << std::endl;

        std::cout << "cameraFrame.frames.size()  is " << cameraFrame.frames.size() << std::endl;

        for (int i = 0; i < cameraFrame.frames.size(); i++) {
            //    DepthPixel* arr = (DepthPixel*)depthFrame.getData();
            if (cameraFrame.frames[i].type == cobotsys::ImageType::Depth) {
                cv::Mat mat = cameraFrame.frames[i].data;
                cv::imshow("Depth", 64*mat);
                cv::waitKey(1);
            }
            if (cameraFrame.frames[i].type == cobotsys::ImageType::Color) {

                cv::Mat mat = cameraFrame.frames[i].data;
                cv::Mat cImageBGR;
                cv::cvtColor(mat, cImageBGR, CV_RGB2BGR);
                cv::imshow("Color", cImageBGR);
                cv::waitKey(1);
            }
        }
    }
};


int main(int argc, char **argv)
{
    QApplication a(argc, argv);
    cobotsys::init_library(argc, argv);
    cobotsys::GlobalObjectFactory globalObjectFactory;
    globalObjectFactory.loadLibrarys();


    auto pFactory = cobotsys::GlobalObjectFactory::instance();
    if (pFactory) {
        auto pobj = pFactory->createObject("FotonicCameraFactory, Ver 1.0", "FotonicCamera");
        std::shared_ptr<cobotsys::AbstractCamera> m_camera = std::dynamic_pointer_cast<cobotsys::AbstractCamera>(pobj);
        if (m_camera) {

            std::cout << "/**************************************/" << std::endl;
            std::cout << "SerialNumber:" << m_camera->getSerialNumber() << std::endl;
            std::cout << "FullDescription:" << m_camera->getFullDescription() << std::endl;
            std::cout << "Manufacturer:" << m_camera->getManufacturer() << std::endl;
            std::cout << "ImageWidth:" << m_camera->getImageWidth() << std::endl;
            std::cout << "/**************************************/" << std::endl;

            bool rc = m_camera->open();

            if (!rc) {
                return -1;
            }

            std::shared_ptr<FotonicObserver> fo = std::make_shared<FotonicObserver>();
            m_camera->attach(fo);
            cv::namedWindow("Depth", CV_WINDOW_AUTOSIZE);
            cv::namedWindow("Color", CV_WINDOW_AUTOSIZE);
            while (1)
                m_camera->capture(1000);


            m_camera->close();
        }
    }

    return 0;
}