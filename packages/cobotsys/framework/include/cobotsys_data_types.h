//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_DATA_TYPES_H
#define COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_DATA_TYPES_H

#include <stdint.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <chrono>

namespace cobotsys {
/**
 * @addtogroup framework
 * @{
 */

/**
 * @brief 几种图像的固定格式
 */
enum class ImageType {
    Color, ///< 彩色BGR图像
    Depth, ///< 深度图像， float, 单位mm
    Ir, ///< 红外图像（Kinect2???
    Mono ///< 单色图像, 灰度图
};

std::string toString(ImageType imageType);

/**
 * @brief 由相机驱动产生捕获的图像
 */
struct ImageFrame {
    ImageType type; ///< 图像数据帧的类型
    const cv::Mat& data; ///< 指向原始数据
};

/**
 * @brief 相机驱动一次捕获操作所获得的图像
 */
struct CameraFrame {
    std::chrono::high_resolution_clock::time_point capture_time; ///< 捕获时间
    std::vector<ImageFrame> frames;/// 捕获的图像
};

/**
 * @brief 无序分拣对应的数据结构
 */
namespace binpicking {
/**
 * @brief 描述在机器人坐标系下，基低坐标系机器人运动的一个状态位置
 */
struct RobotPose {
    cv::Point3d base_position; ///< 相对于机器人的基坐标的位置, 毫米单位
    cv::Vec3d tcp_rpy; ///< 末端安装位置的roll, pitch, yaw, 弧度单位
};

/**
 * @brief 描述由相机，算法模块计算出来的目标信息
 */
struct BinObjGrabPose {
    cv::Point3d position; ///< 相机坐标系下的3D位置
    cv::Vec3d rotation; ///< 相机坐标系下的旋转。
    std::string target_info; ///< 抓取目标类型
};
}



/**
* @brief 六维力数据
*/
struct Wrench {
	cv::Point3d force;
	cv::Point3d torque;
};


/**
 * @}
 */
}

#endif //COBOTSYS_COBOTSYS_ABSTRACT_BINPICKING_DATA_TYPES_H
