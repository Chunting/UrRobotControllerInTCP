//
// Created by zxj on 17-4-24.
// Copyright (c) 2016 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//
#include "cloud_based_plane_segmentation.h"

extern "C" {
#include "lsd.h"
}

namespace cobotsys {
    namespace VisionApi {

        PlaneSegment::PlaneSegment() {
            position_pose_vec.clear();

            min_distance = 0.50;
            max_distance = 0.79;
            sampling_distance = 0.002;
            plane_ditance = 0.002;

        }

        PlaneSegment::PlaneSegment(float minDistance, float maxDistance, float samplingDistance, float planeDistance,
                                   cv::Mat camMatrixColor, cv::Mat camMatrixDepth) :
                min_distance(minDistance), max_distance(maxDistance), sampling_distance(samplingDistance),
                plane_ditance(planeDistance) {

//            fx = 1.0f / (camMatrixColor.at<double>(0, 0) * 512 / 1920);
//            fy = 1.0f / (camMatrixColor.at<double>(1, 1) * 424 / 1080);
//            cx = camMatrixColor.at<double>(0, 2) *  512 / 1920;
//            cy = camMatrixColor.at<double>(1, 2) * 424 / 1080;

            fx = 0.00273197;
            fy = 0.00273197;
            cx = 262.741;
            cy = 206.374;

            position_pose_vec.clear();
        }

        PlaneSegment::~PlaneSegment() {

        }

        ////离散点过滤
        void PlaneSegment::StatisticalOutlierRemovalFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            // Create the filtering object
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud_in);
            sor.setMeanK(50);
            sor.setStddevMulThresh(0);//值越小过滤越明显
            // sor.setNegative (true);//过滤集中点，保留离散点
            sor.filter(*cloud_out);

            std::cerr << "Cloud after StatisticalOutlierRemovalFilter..." << std::endl;
        }


        ////空间区域过滤，过滤超出范围的数据
        void PlaneSegment::passThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            // Create the filtering object
            pcl::PassThrough<pcl::PointXYZRGB> passZ;
            passZ.setInputCloud(cloud_in);
            passZ.setFilterFieldName("z");
            passZ.setFilterLimits(min_distance, max_distance);
            //pass.setFilterLimitsNegative (true);
            passZ.filter(*cloud_out);

            pcl::PassThrough<pcl::PointXYZRGB> passX;
            passX.setInputCloud(cloud_out);
            passX.setFilterFieldName("x");
            passX.setFilterLimits(-0.36, 0.17);
            passX.filter(*cloud_out);

            pcl::PassThrough<pcl::PointXYZRGB> passY;
            passY.setInputCloud(cloud_out);
            passY.setFilterFieldName("y");
            passY.setFilterLimits(-0.15, 0.30);
            passY.filter(*cloud_out);

            std::cerr << "Cloud after passThroughFilter..." << std::endl;
        }

        ////点云数据采样，减少点云数据量
        void PlaneSegment::voxelGridFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            // Create the filtering object
            pcl::VoxelGrid<pcl::PointXYZRGB> sor;
            sor.setInputCloud(cloud_in);
            sor.setLeafSize(sampling_distance, sampling_distance, sampling_distance);
            sor.filter(*cloud_out);

            std::cerr << "Cloud after voxelGridFilter..." << std::endl;
        }

        ////Projecting points using a parametric model
        ////We fill in the ModelCoefficients values. In this case, we use a plane model,
        ////with ax+by+cz+d=0, where a=b=d=0, and c=1, or said differently, the X-Y plane.
        void PlaneSegment::ModelCoefficientsFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                                   Eigen::VectorXf normal,
                                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            // Create a set of planar coefficients with X=Y=0,Z=1
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
            coefficients->values.resize(4);
            coefficients->values[0] = normal[0];
            coefficients->values[1] = normal[1];
            coefficients->values[2] = normal[2];
            coefficients->values[3] = 0;

            // Create the filtering object
            pcl::ProjectInliers<pcl::PointXYZRGB> proj;
            proj.setModelType(pcl::SACMODEL_PLANE);
            proj.setInputCloud(cloud_in);
            proj.setModelCoefficients(coefficients);
            proj.filter(*cloud_out);
        }


        ////区域生长分割
        void PlaneSegment::regionGrowingSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color,
                                                     std::vector<pcl::PointIndices> &clusters) {

            pcl::search::Search<pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(
                    new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
            normal_estimator.setSearchMethod(tree);
            normal_estimator.setInputCloud(cloud_in);
            normal_estimator.setKSearch(50);
            normal_estimator.compute(*normals);

            pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
            reg.setMinClusterSize(50);
            reg.setMaxClusterSize(1000000);
            reg.setSearchMethod(tree);
            reg.setNumberOfNeighbours(30);

            reg.setInputCloud(cloud_in);
            reg.setInputNormals(normals);
            reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
            reg.setCurvatureThreshold(1.0);
            reg.extract(clusters);

            cloud_color = reg.getColoredCloud();

            std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
            std::cerr << "Cloud after regionGrowingSegmentation... " << std::endl;
        }

        ////平面拟合
        ////返回平面法向量以及拟合的平面点云
        Eigen::Vector3d PlaneSegment::planeFitting(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in,
                                                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            std::vector<int> inliers;

            // created RandomSampleConsensus object and compute the appropriated model
            pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr
                    model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>(cloud_in));

            pcl::RandomSampleConsensus<pcl::PointXYZRGB> ransac(model_p);
            ransac.setDistanceThreshold(plane_ditance);
            ransac.computeModel();
            ransac.getInliers(inliers);

            Eigen::VectorXf model_coeff;
            ransac.getModelCoefficients(model_coeff);

            if (model_coeff[2] < 0) {
                model_coeff = -model_coeff;
            }

            // copies all inliers of the model computed to another PointCloud
            pcl::copyPointCloud<pcl::PointXYZRGB>(*cloud_in, inliers, *cloud_out);

            std::cout << "planeNorm:"
                      << " ," << std::setw(10) << model_coeff[0]
                      << " ," << std::setw(10) << model_coeff[1]
                      << " ," << std::setw(10) << model_coeff[2];

            Eigen::Vector3d pose;
            pose[0] = model_coeff[0];
            pose[1] = model_coeff[1];
            pose[2] = model_coeff[2];

            return pose;
        }

        ////计算平面法向量
        void
        PlaneSegment::planeNormal(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, Eigen::Vector3d &planeNorm) {
            pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
            ne.setInputCloud(cloud);

            boost::shared_ptr<pcl::search::KdTree<pcl::PointXYZRGB> > tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
            ne.setSearchMethod(tree);

            boost::shared_ptr<pcl::PointCloud<pcl::Normal> > cloud_normals(new pcl::PointCloud<pcl::Normal>);
            ne.setRadiusSearch(0.5);
            ne.compute(*cloud_normals);

            //ne.computePointNormal();//计算某个点的法向量

            planeNorm[0] = cloud_normals->at(0).normal_x;
            planeNorm[1] = cloud_normals->at(0).normal_y;
            planeNorm[2] = cloud_normals->at(0).normal_z;
            if (planeNorm[2] < 0) {
                planeNorm = -planeNorm;
            }
            planeNorm = planeNorm / planeNorm.norm();

            std::cout << "planeNorm:"
                      << " ," << std::setw(10) << planeNorm[0]
                      << " ," << std::setw(10) << planeNorm[1]
                      << " ," << std::setw(10) << planeNorm[2] << std::endl;
        }

        ////本地读取点云数据
        void PlaneSegment::loadPCDFile(const std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

            pcl::io::loadPCDFile(filename, *cloud);
            std::cout << "width" << cloud->height << ":" << cloud->width << std::endl;
        }

        ////点云显示
        void PlaneSegment::cloudViewer(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
//            const std::string cloudName = "box_cloud";
//            pcl::visualization::PCLVisualizer viewer("Viewer");
//            viewer.addPointCloud(cloud);
//            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
//            viewer.initCameraParameters();
//            viewer.setBackgroundColor(255, 255, 255);
//            viewer.setPosition(0, 0);
//            viewer.setSize(cloud->width, cloud->height);
//            viewer.setShowFPS(true);
//            viewer.setCameraPosition(0, 0, 0, 0, -1, 0);
//
//            while (!viewer.wasStopped()) {
//                viewer.spinOnce();
//            }
        }


        ////沿法线方向投影到平面，拟合工件最小外接矩形
        ////计算工件中心
        void getObjectCenter() {
            //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
            //ModelCoefficientsFilter(object_plane,object_pose.pose,cloud_out);
        }

        ////获取平面中心坐标及法向量
        void PlaneSegment::getObejectPositionPose(cv::Mat &color, const std::vector<pcl::PointIndices> clusters,
                                                  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_color,
                                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out) {
            cloud_out->clear();
            position_pose_vec.clear();
            if (clusters.empty() || cloud_color->empty()) {
                std::cerr << "COBOT WARNING! None object detected!" << std::endl;
            }
            for (int counter = 0; counter < clusters.size(); counter++) {
                if (clusters[counter].indices.size() < 50)//过滤小的平面
                {
                   // continue;
                }
                PositionPose position_pose;
                position_pose.score = clusters[counter].indices.size();//拟合平面点数（完整度）
                std::cout << std::setw(4) << clusters[counter].indices.size() << " ,";
                if (counter % 10 == 0 && counter != 0)
                    std::cout << std::endl;

                //平面拟合，获取平面法向量
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_plane(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*cloud_color, clusters[counter], *object_plane);
                position_pose.pose = planeFitting(object_plane, object_plane);//平面法向量

                Eigen::Vector3d vz(0, 0, 1);//TODO 法向量角度判断
                double Ndegree = acos(vz.dot(position_pose.pose) / position_pose.pose.norm()) * 180 / CV_PI;
                std::cout << "------>法向量和z轴的夹角:" << Ndegree << std::endl;
                if (Ndegree > 45)//角度限制
                {
                    std::cerr << "ERROR: Ndegree > 45" << std::endl;
                    continue;
                }

                cv::Mat object_mask = cv::Mat(color.rows, color.cols, CV_8UC1, cv::Scalar(0));
                cloud2Image(cloud_color, clusters[counter], object_mask);
//                cv::imshow("object", object_mask);

                std::vector<cv::Point> centers;
                ConnectedObjectSegmentation(color, object_mask, centers);


                //获取工件中心,
                //TODO 需优化
                for (int i = 0; i < clusters[counter].indices.size(); i++) {
                    //position_pose.position.x += cloud_color->points[clusters[counter].indices[i]].x;
                    //position_pose.position.y += cloud_color->points[clusters[counter].indices[i]].y;
                    position_pose.position.z += cloud_color->points[clusters[counter].indices[i]].z;
                }
                //position_pose.position.x = position_pose.position.x / clusters[counter].indices.size();
                //position_pose.position.y = position_pose.position.y / clusters[counter].indices.size();
                position_pose.position.z = position_pose.position.z / clusters[counter].indices.size();

                for (int num = 0; num < centers.size(); num++) {
                    float depthValue = position_pose.position.z;
                    position_pose.position.x = (centers[num].x - cx) * fx * depthValue;
                    position_pose.position.y = (centers[num].y - cy) * fy * depthValue;
                    //position_pose.position.z = depthValue;
                    position_pose_vec.push_back(position_pose);
                }
                cloud_out->operator+=(*object_plane);//拟合平面合并　
            }

            //按轮廓大小排序，优先抓取最外面的
            std::sort(position_pose_vec.begin(), position_pose_vec.end(),
                      [=](const PositionPose &pp1, const PositionPose &pp2) {
                          return pp1.score > pp2.score;
                      });
            //按高度排序,优先抓取上面的
            std::sort(position_pose_vec.begin(), position_pose_vec.end(),
                      [=](const PositionPose &pp1, const PositionPose &pp2) {
                          return pp1.position.z < pp2.position.z;
                      });
        }

        void PlaneSegment::processing(cv::Mat &color, cv::Mat depth) {

            if(color.empty() || depth.empty())
            {
                return;
            }
            cv::Mat gray, color_back;
            color_back = color.clone();
            cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
            getLinesWithLSD(gray);
            drawSegments(color_back);

            cv::RotatedRect boxRect;
            //extractBoxEdge(color,boxRect);
            extractBoxEdge2(color, depth, boxRect);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
            cloud->height = color.rows;
            cloud->width = color.cols;
            cloud->is_dense = false;
            cloud->points.resize(cloud->height * cloud->width);
            createLookup(color.cols, color.rows);
            createCloud(depth,color,cloud);

#ifdef DEBUG
            cv::Mat image = cv::Mat(color.rows, color.cols, CV_8UC1, cv::Scalar(0));
            cloud2Image(cloud,image);
            cv::imshow("cloudImage",image);
#endif


            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pass(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_grid(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);
            std::vector<pcl::PointIndices> clusters;

            passThroughFilter(cloud, cloud_pass);//X.Y.Z轴空间范围过滤
//            cloudViewer(cloud_pass);

            StatisticalOutlierRemovalFilter(cloud_pass, cloud_filt);//统计过滤
//            cloudViewer(cloud_filt);

            voxelGridFilter(cloud_filt, cloud_grid);//样本抽样
//            cloudViewer(cloud_grid);

            regionGrowingSegmentation(cloud_grid, cloud_color, clusters);//平面分割
//            cloudViewer(cloud_color);

//            cv::Mat image = cv::Mat(color.rows, color.cols, CV_8UC1, cv::Scalar(0));
//            cloud2Image(cloud,image);
//            cv::imshow("cloudImage",image);

            getObejectPositionPose(color, clusters, cloud_color, cloud_out);//拟合每个平面并获取面法向量
//            cloudViewer(cloud_out);
        }

        //点云数据转成图像
        void PlaneSegment::cloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                       pcl::PointIndices pointIndices, cv::Mat &object) {

            if (cloud->empty() || object.empty()) {
                std::cerr << "cloud2Image: NULL ptr error!" << std::endl;
                return;
            }
            #pragma omp parallel for
            for (int i = 0; i < pointIndices.indices.size(); i++) {
                cv::Point pt;
                pcl::PointXYZRGB point = cloud->points[pointIndices.indices[i]];
                pt.x = (point.x / point.z) / fx + cx;
                pt.y = (point.y / point.z) / fy + cy;

                cv::circle(object, pt, 1, cv::Scalar(255), 2);
            }
        }

        void PlaneSegment::cloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat &image)
        {
            if (cloud->empty() || image.empty()) {
                std::cerr << "cloud2Image: NULL ptr error!" << std::endl;
                return;
            }
            #pragma omp parallel for
            for (int i = 0; i < cloud->points.size(); i++) {
                cv::Point pt;
                pcl::PointXYZRGB point = cloud->points[i];
                pt.x = (point.x / point.z) / fx + cx;
                pt.y = (point.y / point.z) / fy + cy;

                cv::circle(image, pt, 1, cv::Scalar(255), 1);
            }
        }
        //连接工件分割
        bool PlaneSegment::ConnectedObjectSegmentation(cv::Mat &image, cv::Mat &mask,
                                                       std::vector<cv::Point> &centers) {

            //去掉面积较小区域
//            cv::Mat element_open = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(15, 15));
//            cv::Mat element_close = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(3, 3));
//            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element_close);
//            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element_open);
//
//            for (int i = 0; i < segments.size(); i++) {
//                cv::Point2i pt0, pt1;
//                const cv::Vec4d &s = segments[i];
//                pt0 = cv::Point(round(s[0]), round(s[1]));
//                pt1 = cv::Point(round(s[2]), round(s[3]));
//                cv::line(mask, pt0, pt1, cv::Scalar(0), 2);
//            }
//            cv::imshow("seg", mask);

            std::vector<std::vector<cv::Point>> contours;
            findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

            cv::Point2f P[4];
            std::vector<std::vector<cv::Point>>::iterator it = contours.begin();

            while (it != contours.end()) {
                cv::RotatedRect rRec = cv::minAreaRect(*it); //获取最小外接矩形
                rRec.points(P);
                if (rRec.size.width < 10 || rRec.size.height < 10) {
                    it++;
                    continue;
                }
                for (int j = 0; j < 4; j++) {
                    cv::line(image, P[j], P[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                }
                std::cout << "width:" << rRec.size.width << " ---height:" << rRec.size.height << std::endl;
                if (rRec.size.width > 40 && rRec.size.height > 40) //宽:32pix 长:80pix
                {
                    cv::Point pt1, pt2;
                    cv::Point center1, center2;

                    if (rRec.size.width < rRec.size.height) {
                        pt1 = cv::Point2f((P[1].x + P[2].x) / 2, (P[1].y + P[2].y) / 2);
                        pt2 = cv::Point2f((P[0].x + P[3].x) / 2, (P[0].y + P[3].y) / 2);
                        center1 = cv::Point2f((P[2].x + P[3].x + pt1.x + pt2.x) / 4,
                                              (P[2].y + P[3].y + pt1.y + pt2.y) / 4);
                        center2 = cv::Point2f((P[0].x + P[1].x + pt1.x + pt2.x) / 4,
                                              (P[0].y + P[1].y + pt1.y + pt2.y) / 4);

                    } else {
                        pt1 = cv::Point2f((P[0].x + P[1].x) / 2, (P[0].y + P[1].y) / 2);
                        pt2 = cv::Point2f((P[2].x + P[3].x) / 2, (P[2].y + P[3].y) / 2);
                        center1 = cv::Point2f((P[1].x + P[2].x + pt1.x + pt2.x) / 4,
                                              (P[1].y + P[2].y + pt1.y + pt2.y) / 4);
                        center2 = cv::Point2f((P[0].x + P[3].x + pt1.x + pt2.x) / 4,
                                              (P[0].y + P[3].y + pt1.y + pt2.y) / 4);
                    }
                    centers.push_back(center1);//TODO 　待优化
                    centers.push_back(center2);
                    cv::line(image, pt1, pt2, cv::Scalar(0, 255, 0), 2);
                    cv::circle(image, center1, 2, cv::Scalar(0, 255, 0), 2);
                    cv::circle(image, center2, 2, cv::Scalar(0, 255, 0), 2);

                } else {
                    centers.push_back(rRec.center);
                    cv::circle(image, rRec.center, 2, cv::Scalar(0, 255, 0), 2);
                }

                it++;
            }
        }

        //图像转点云数据
        void PlaneSegment::createCloud(const cv::Mat &depth, const cv::Mat &color,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
            const float badPoint = std::numeric_limits<float>::quiet_NaN();

            #pragma omp parallel for
            for (int r = 0; r < depth.rows; ++r) {
                pcl::PointXYZRGB *itP = &cloud->points[r * depth.cols];
                const uint16_t *itD = depth.ptr<uint16_t>(r);
                const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
                const float y = lookupY.at<float>(0, r);
                const float *itX = lookupX.ptr<float>();

                for (size_t c = 0; c < (size_t) depth.cols; ++c, ++itP, ++itD, ++itC, ++itX) {
                    register const float depthValue = *itD / 1000.0f;
                    // Check for invalid measurements
                    if (*itD == 0) {
                        // not valid
                        itP->x = itP->y = itP->z = badPoint;
                        //itP->rgb = 0;
                        continue;
                    }
                    itP->z = depthValue;
                    itP->x = *itX * depthValue;
                    itP->y = y * depthValue;
                    itP->b = itC->val[0];
                    itP->g = itC->val[1];
                    itP->r = itC->val[2];
                }
            }
        }

        void PlaneSegment::createLookup(size_t width, size_t height) {
            float *it;
            lookupY = cv::Mat(1, height, CV_32F);
            it = lookupY.ptr<float>();
            for (size_t r = 0; r < height; ++r, ++it) {
                *it = (r - cy) * fy;
            }

            lookupX = cv::Mat(1, width, CV_32F);
            it = lookupX.ptr<float>();
            for (size_t c = 0; c < width; ++c, ++it) {
                *it = (c - cx) * fx;
            }
        }

        void PlaneSegment::getLinesWithLSD(cv::Mat src_img) {

            double *dbl_img = new double[src_img.cols * src_img.rows];

            for (int y = 0; y < src_img.rows; y++)
                for (int x = 0; x < src_img.cols; x++)
                    dbl_img[x + y * src_img.cols] = (double) src_img.at<uchar>(y, x);

            int n_segments;

            double *dbl_segments = LineSegmentDetection(&n_segments, dbl_img, src_img.cols, src_img.rows,
                                                        1.0 /*scale*/,  //0.8
                                                        1.0 /*sigma_scale*/,  //0.6
                                                        2.0 /* quant*/,   //2.0  0.5
                                                        22.5 /* ang_th */,//22.5
                                                        0.0 /* log_eps */, //1.0 2.0 -1.0
                                                        0.3 /* density_th */,  //0.7  0.5
                                                        1024 /* n_bins */,
                                                        NULL, NULL, NULL);

            delete[] dbl_img;
            segments.clear();

            for (int i = 0; i < n_segments; i++) {
                cv::Vec4d s;
                s[0] = dbl_segments[7 * i];
                s[1] = dbl_segments[7 * i + 1];
                s[2] = dbl_segments[7 * i + 2];
                s[3] = dbl_segments[7 * i + 3];

                float length = std::sqrt((s[0] - s[2]) * (s[0] - s[2]) + (s[1] - s[3]) * (s[1] - s[3]));
                if (length < 20 || length > 100) {
                    //continue;
                }
                //线段延长
//                s[0] = s[0] + (s[0] - s[2])/2;
//                s[1] = s[1] + (s[1] - s[3])/2;
//                s[2] = s[2] + (s[2] - s[0])/2;
//                s[3] = s[3] + (s[3] - s[1])/2;

                segments.push_back(s);
            }
            delete dbl_segments;
        }

        void PlaneSegment::drawSegments(cv::Mat &dbg_img) {
            //画出所有线段
            for (int i = 0; i < segments.size(); i++) {
                cv::Point2i pt0, pt1;
                const cv::Vec4d &s = segments[i];
                pt0 = cv::Point(round(s[0]), round(s[1]));
                pt1 = cv::Point(round(s[2]), round(s[3]));
                cv::line(dbg_img, pt0, pt1, CV_RGB(random() % 256, random() % 256, random() % 256), 1);
                cv::circle(dbg_img, pt0, 2, CV_RGB(255, 0, 0), 1);
                cv::circle(dbg_img, pt1, 2, CV_RGB(255, 0, 0), 1);
            }

//            cv::imshow("LSD", dbg_img);
        }

        //提取料框边缘  OSTU 分割
        bool PlaneSegment::extractBoxEdge(cv::Mat &color, cv::RotatedRect &boxRect) {
            cv::Mat hsv_color;
            cv::cvtColor(color, hsv_color, CV_BGR2HSV);
            std::vector<cv::Mat> hsv;
            cv::split(hsv_color, hsv);

            cv::Mat minH, maxH, thredS, thredV;
            cv::threshold(hsv[0], minH, 34, 255, CV_THRESH_BINARY);//蓝色区间  100 -124（max180）
            cv::threshold(hsv[0], maxH, 11, 255, CV_THRESH_BINARY);
            cv::threshold(hsv[1], thredS, 0, 255, CV_THRESH_OTSU);
            cv::threshold(hsv[2], thredV, 0, 255, CV_THRESH_OTSU);

            //蓝色H、S、V合并
            cv::Mat box, temp;
            cv::bitwise_not(minH, minH);
            cv::bitwise_and(maxH, minH, box);
            cv::bitwise_and(thredS, thredV, temp);
            cv::bitwise_and(temp, box, box);

            imshow("料框", box);

            std::vector<std::vector<cv::Point>> contours;
            //只检测外轮廓
            findContours(box, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
            if (contours.size() == 0) {
                std::cout << "Can not locate the box. Try again!!!";
                return false;
            }

            std::sort(contours.begin(), contours.end(),
                      [=](const std::vector<cv::Point> &vp1, const std::vector<cv::Point> &vp2) {
                          return vp1.size() > vp2.size();
                      });

            cv::Point2f P[4];
            boxRect = cv::minAreaRect(contours[0]); //获取最小外接矩形

            boxRect.points(P);
            cv::Point2f insideP[4];
            for (int i = 0; i < 4; i++) {//料框边缘宽度占料框宽度一半的1/10
                insideP[i] = cv::Point2f(P[i].x + (boxRect.center.x - P[i].x) * 0.1,
                                         P[i].y + (boxRect.center.y - P[i].y) * 0.1);
            }
            for (int j = 0; j < 4; j++) {
                cv::line(color, P[j], P[(j + 1) % 4], cv::Scalar(0, 0, 255), 2);
                cv::line(color, insideP[j], insideP[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
                cv::line(color, P[j], insideP[j], cv::Scalar(0, 255, 0), 2);
            }

//            cv::imshow("料框边缘", color);
            return true;

        }

        //提取料框边缘(深度)
        bool PlaneSegment::extractBoxEdge2(cv::Mat &color, const cv::Mat depth, cv::RotatedRect &boxRect) {

            cv::Mat depth8U = cv::Mat(depth.rows, depth.cols, CV_8UC1);
            for (int i = 0; i < depth.rows; i++) {
                for (int j = 0; j < depth.cols; j++) {
                    if (depth.at<ushort>(i, j) < 600 && depth.at<ushort>(i, j) > 500) {
                        depth8U.at<uchar>(i, j) = 255;
                    } else {
                        depth8U.at<uchar>(i, j) = 0;
                    }
                }
            }
            //cv::imshow("depth", depth8U);

            std::vector<std::vector<cv::Point>> contours;
            //只检测外轮廓
            findContours(depth8U, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
            if (contours.size() == 0) {
                std::cout << "Can not locate the box. Try again!!!";
                return false;
            }

            std::sort(contours.begin(), contours.end(),
                      [=](const std::vector<cv::Point> &vp1, const std::vector<cv::Point> &vp2) {
                          return vp1.size() > vp2.size();
                      });

            cv::Point2f P[4];
            boxRect = cv::minAreaRect(contours[0]); //获取最小外接矩形

            cv::drawContours(color,contours,0,cv::Scalar(255,0,0),2);
            boxRect.points(P);
            cv::Point2f insideP[4];
            for (int i = 0; i < 4; i++) {//料框边缘宽度占料框宽度一半的1/10
                insideP[i] = cv::Point2f(P[i].x + (boxRect.center.x - P[i].x) * 0.1,
                                         P[i].y + (boxRect.center.y - P[i].y) * 0.1);
            }
            for (int j = 0; j < 4; j++) {
                cv::line(color, P[j], P[(j + 1) % 4], cv::Scalar(0, 0, 255), 1);
                cv::line(color, insideP[j], insideP[(j + 1) % 4], cv::Scalar(0, 255, 0), 1);
                cv::line(color, P[j], insideP[j], cv::Scalar(0, 255, 0), 1);
            }

            //cv::imshow("料框边缘", color);
            return true;

        }
    }
}
