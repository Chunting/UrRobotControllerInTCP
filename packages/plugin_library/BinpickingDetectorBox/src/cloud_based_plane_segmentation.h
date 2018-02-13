//
// Created by zxj on 17-4-24.
// Copyright (c) 2016 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_CLOUD_BASED_PLANE_SEGMENTATION_H
#define PROJECT_CLOUD_BASED_PLANE_SEGMENTATION_H

#include <iostream>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

//#define DEBUG true

namespace cobotsys {
    namespace VisionApi {

        ////工件空间位置及姿态
        struct PositionPose {
            Eigen::Vector3d pose;//工件法向量
            pcl::PointXYZ position;//工件中心点
            float score;//工件评分
        };

        class PlaneSegment {

        public:
            PlaneSegment(float minDistance, float maxDistance, float samplingDistance, float planeDistance,
                         cv::Mat camMatrixColor, cv::Mat camMatrixDepth);

            PlaneSegment();

            ~PlaneSegment();

            void StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

            void passThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

            void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                 pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

            void ModelCoefficientsFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, Eigen::VectorXf normal,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);


            void regionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color,
                                           std::vector<pcl::PointIndices> &clusters);


            Eigen::Vector3d planeFitting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

            void planeNormal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Vector3d &planeNorm);

            void loadPCDFile(const std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

            void cloudViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

            void getObejectPositionPose(cv::Mat &color, const std::vector<pcl::PointIndices> clusters,
                                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color,
                                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);

            void processing(cv::Mat &color, cv::Mat depth);

            void cloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointIndices pointIndices,
                             cv::Mat &object);

            void cloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat &image);

            bool ConnectedObjectSegmentation(cv::Mat &image, cv::Mat &mask, std::vector<cv::Point> &centers);
            
            void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

            void createLookup(size_t width, size_t height);

            void getLinesWithLSD(cv::Mat src_img);

            void drawSegments(cv::Mat &dbg_img);

            bool extractBoxEdge(cv::Mat &color, cv::RotatedRect &boxRect);

            bool extractBoxEdge2(cv::Mat &color,const cv::Mat depth, cv::RotatedRect &boxRect);

        protected:
            void getObjectCenter();

        public:
            std::vector<PositionPose> position_pose_vec;//所有识别到的工件空间位置及姿态

        private:
            std::vector<cv::Vec4d> segments;

            float min_distance = 0.50;//工件到相机最小距离   单位：米

            float max_distance = 0.76;//工件到相机最远距离

            float sampling_distance = 0.003;//点云采样距离

            float plane_ditance = 0.002;//平面拟合采样距离

            float fx ,fy, cx, cy;

            cv::Mat lookupX, lookupY;
        };
    }
}

#endif //PROJECT_CLOUD_BASED_PLANE_SEGMENTATION_H
