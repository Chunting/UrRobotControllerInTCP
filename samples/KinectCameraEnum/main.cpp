//
// Created by 潘绪洋 on 17-3-3.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <iostream>
#include <cobotsys_logger.h>
#include <unistd.h>
#include <chrono>

int main(int argc, char** argv){
    /// [context]
    libfreenect2::Freenect2 freenect2;
    /// [context]

    std::string serial = "";

    bool viewer_enabled = true;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;

    int loop_time = 0;
    while (loop_time < 10) {
        auto time_start = std::chrono::high_resolution_clock::now();
        int num_dev = freenect2.enumerateDevices();
        auto time_stop = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double> diff = time_stop - time_start;


        if (num_dev > 0) {
            break;
        }
        COBOT_LOG.warning() << "No Kinect2 Device : " << loop_time++ << ", " << diff.count();
        usleep(1000000);
    }


    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }
    /// [discovery]

    return 0;
}