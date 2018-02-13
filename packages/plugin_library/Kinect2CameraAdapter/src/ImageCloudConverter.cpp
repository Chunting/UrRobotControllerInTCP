//
// Created by vimjian on 17-4-28.
//

#include "ImageCloudConverter.h"


ImageCloudConverter::ImageCloudConverter() {
/*    color_fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
    color_fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
    color_cx = cameraMatrixColor.at<double>(0, 2);
    color_cy = cameraMatrixColor.at<double>(1, 2);

    depth_fx = 1.0f / cameraMatrixDepth.at<double>(0, 0);
    depth_fy = 1.0f / cameraMatrixDepth.at<double>(1, 1);
    depth_cx = cameraMatrixDepth.at<double>(0, 2);
    depth_cy = cameraMatrixDepth.at<double>(1, 2);*/

mode=HD; //default

}

/*
void ImageCloudConverter::ConvertPixel_depthToCloud(const Point2i &depthPixel, Point3f &cloudPoint) {
    float depthValue= depth.at<uint16_t>(depthPixel.x, depthPixel.y) / 1.000f;//  m
    float pointx =(depthPixel.x-depth_cx)*depth_fx*depthValue;
    float pointy =(depthPixel.y-depth_cy)*depth_fx*depthValue;

    cloudPoint.x = pointx;
    cloudPoint.y = pointy;
    cloudPoint.z = depthValue;
}

void ImageCloudConverter::ConvertPoint_cloudToDepth(const Point3f &cloudPoint, Point2i &depthPixel) {
    depthPixel.x=cloudPoint.x/(cloudPoint.z*depth_fx)+depth_cx;
    depthPixel.y=cloudPoint.y/(cloudPoint.z*depth_fy)+depth_cy;
}
*/

void ImageCloudConverter::init()
{
    if (path_yaml=="") {
        path_yaml = "/home/vimjian/CLionProjects/cobotsys/packages/plugin_library/Kinect2CameraAdapter/src/017102364847/";
        cout<<"using default yaml files path!!!"<<endl;
    }

    //初始化相机参数

    if(!loadCalibrationFile(path_yaml + K2_CALIB_COLOR , cameraMatrixColor, distortionColor))
    {
        printf("using sensor defaults for color intrinsic parameters.");
    }

    if( !loadCalibrationFile(path_yaml + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
    {
       printf("using sensor defaults for ir intrinsic parameters.");
    }

    if(!loadCalibrationPoseFile(path_yaml + K2_CALIB_POSE  , rotation, translation))
    {
        printf("using defaults for rotation and translation.");
    }

    if( !loadCalibrationDepthFile(path_yaml + K2_CALIB_DEPTH, depthShift))
    {
        printf("using defaults for depth shift.");
        depthShift = 0.0;
    }

    zNear=0.5;
    zFar=12;

    cameraMatrix_qhd=cameraMatrixColor.clone();
    cameraMatrix_qhd.at<double>(0, 0) /= 2;
    cameraMatrix_qhd.at<double>(1, 1) /= 2;
    cameraMatrix_qhd.at<double>(0, 2) /= 2;
    cameraMatrix_qhd.at<double>(1, 2) /= 2;


    //cameraMatrixRegistered   for  color
    if (mode==HD) {
        sizeRegistered = Size(1920, 1080);
        cameraMatrixRegistered=cameraMatrixColor.clone();
    }
    else if(mode==QHD)
    {
        sizeRegistered = Size(1920/2, 1080/2);
        cameraMatrixRegistered=cameraMatrix_qhd.clone();
    }

    fx = cameraMatrixRegistered.at<double>(0, 0);
    fy = cameraMatrixRegistered.at<double>(1, 1);
    cx = cameraMatrixRegistered.at<double>(0, 2) + 0.5;
    cy = cameraMatrixRegistered.at<double>(1, 2) + 0.5;


    //rectify map
    initRectify();
    //project Matrix
    proj(0, 0) = rotation.at<double>(0, 0);
    proj(0, 1) = rotation.at<double>(0, 1);
    proj(0, 2) = rotation.at<double>(0, 2);
    proj(0, 3) = translation.at<double>(0, 0);
    proj(1, 0) = rotation.at<double>(1, 0);
    proj(1, 1) = rotation.at<double>(1, 1);
    proj(1, 2) = rotation.at<double>(1, 2);
    proj(1, 3) = translation.at<double>(1, 0);
    proj(2, 0) = rotation.at<double>(2, 0);
    proj(2, 1) = rotation.at<double>(2, 1);
    proj(2, 2) = rotation.at<double>(2, 2);
    proj(2, 3) = translation.at<double>(2, 0);
    proj(3, 0) = 0;
    proj(3, 1) = 0;
    proj(3, 2) = 0;
    proj(3, 3) = 1;

    //lookup
    createLookup();
    /*

    cout<<"matrixColor="<<cameraMatrixColor<<endl;
    cout<<"distortColor="<<distortionColor<<endl;
    cout<<"matrixDepth="<<cameraMatrixDepth<<endl;
    cout<<"distortDepth="<<distortionDepth<<endl;

*/

}

bool
ImageCloudConverter::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
        fs[K2_CALIB_DISTORTION] >> distortion;
        fs.release();
    }
    else
    {
        std::cerr<<"can't open calibration file: " << filename<<endl;
        return false;
    }
    return true;
}

bool
ImageCloudConverter::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_ROTATION] >> rotation;
        fs[K2_CALIB_TRANSLATION] >> translation;
        fs.release();
    }
    else
    {
        std::cerr<<"can't open calibration pose file: " << filename<<std::endl;
        return false;
    }
    return true;
}

bool ImageCloudConverter::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
    cv::FileStorage fs;
    if(fs.open(filename, cv::FileStorage::READ))
    {
        fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
        fs.release();
    }
    else
    {
        std::cerr<<"can't open calibration depth file: " << filename<<endl;
        return false;
    }
    return true;
}

void ImageCloudConverter::registerDepth(cv::Mat &registeredDepth,cv::Mat &registeredColor)
{
    rectifyColor();
    cv::Mat scaled;
    remapDepth(depth, scaled);
  //  cv::imshow("scale",64*scaled);
    projectDepth(scaled, registeredDepth,registeredColor);
  //  cv::imshow("reg",64*registeredDepth);
  //  cv::imshow("regColor",registeredColor);
    medianBlur(registeredDepth,registeredDepth,3);
    if (mode==HD)
    registeredColor=color_rect;
    else if (mode==QHD)
    {
        registeredColor=color_qhd_rect;
    }
  //  cv::imshow("regDepth",64*registeredDepth);
  //  cv::imshow("regColor",registeredColor);
  //  cout<<"register One!"<<endl;
}

void ImageCloudConverter::remapDepth(const cv::Mat &depth, cv::Mat &scaled) const
{
    /*cv::Mat xx= mapX(cv::Range(150,160),cv::Range(150,160));
    std::cout<<"xx=="<<xx<<std::endl;
    cv::Mat yy= mapY(cv::Range(150,160),cv::Range(150,160));
    std::cout<<"yy=="<<yy<<std::endl;*/

    scaled.create(sizeRegistered, CV_16U);
#pragma omp parallel for
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
    {
        uint16_t *itO = scaled.ptr<uint16_t>(r);
        const float *itX = mapX_depth.ptr<float>(r);
        const float *itY = mapY_depth.ptr<float>(r);
        for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itO, ++itX, ++itY)
        {
            *itO = interpolate(depth, *itX, *itY);
        }
    }
}

void ImageCloudConverter::projectDepth(const cv::Mat &scaled, cv::Mat &registeredDepth,cv::Mat &registeredColor) const
{
    registeredDepth = cv::Mat::zeros(sizeRegistered, CV_16U);
    registeredColor = cv::Mat::zeros(sizeRegistered, CV_16UC3);

    /*for (int i = 50; i <60 ; ++i) {
        std::cout<<"ly="<<lookupY.at<double>(0, i)<<endl;
        std::cout<<"scale="<<scaled.at<uint16_t>(i, 93)<<std::endl;

    }*/
#pragma omp parallel for
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r)
    {
        const uint16_t *itD = scaled.ptr<uint16_t>(r);
        const double y = lookupY.at<double>(0, r);
        const double *itX = lookupX.ptr<double>();

        for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++itD, ++itX)
        {
            const double depthValue = *itD / 1000.0;
            if(depthValue < zNear || depthValue > zFar)
            {
                continue;
            }

            Eigen::Vector4d pointD(*itX * depthValue, y * depthValue, depthValue, 1);
            Eigen::Vector4d pointP = proj * pointD;

            const double z = pointP[2];

            const double invZ = 1 / z;
            const int xP = (fx * pointP[0]) * invZ + cx;
            const int yP = (fy * pointP[1]) * invZ + cy;

            if(xP >= 0 && xP < sizeRegistered.width && yP >= 0 && yP < sizeRegistered.height)
            {
                const uint16_t z16 = z * 1000;
                uint16_t &zReg = registeredDepth.at<uint16_t>(yP, xP);
                if(zReg == 0 || z16 < zReg)
                {
                    zReg = z16;
                }

               /* registeredColor.at<Vec3b>(yP,xP)[0]=color_qhd.at<Vec3b>(yP,xP)[0];
                registeredColor.at<Vec3b>(yP,xP)[1]=color_qhd.at<Vec3b>(yP,xP)[1];
                registeredColor.at<Vec3b>(yP,xP)[2]=color_qhd.at<Vec3b>(yP,xP)[2];*/
            }
        }
    }

}

 uint16_t ImageCloudConverter::interpolate(const cv::Mat &in, const float &x, const float &y) const //
{
    const int xL = (int)floor(x);
    const int xH = (int)ceil(x);
    const int yL = (int)floor(y);
    const int yH = (int)ceil(y);

    if(xL < 0 || yL < 0 || xH >= in.cols || yH >= in.rows)
    {
        return 0;
    }

    const uint16_t pLT = in.at<uint16_t>(yL, xL);
    const uint16_t pRT = in.at<uint16_t>(yL, xH);
    const uint16_t pLB = in.at<uint16_t>(yH, xL);
    const uint16_t pRB = in.at<uint16_t>(yH, xH);
    int vLT = pLT > 0;
    int vRT = pRT > 0;
    int vLB = pLB > 0;
    int vRB = pRB > 0;
    int count = vLT + vRT + vLB + vRB;

    if(count < 3)
    {
        return 0;
    }

    const uint16_t avg = (pLT + pRT + pLB + pRB) / count;
    const uint16_t thres = 0.01 * avg;
    vLT = abs(pLT - avg) < thres;
    vRT = abs(pRT - avg) < thres;
    vLB = abs(pLB - avg) < thres;
    vRB = abs(pRB - avg) < thres;
    count = vLT + vRT + vLB + vRB;

    if(count < 3)
    {
        return 0;
    }

    double distXL = x - xL;
    double distXH = 1.0 - distXL;
    double distYL = y - yL;
    double distYH = 1.0 - distYL;
    distXL *= distXL;
    distXH *= distXH;
    distYL *= distYL;
    distYH *= distYH;
    const double tmp = sqrt(2.0);
    const double fLT = vLT ? tmp - sqrt(distXL + distYL) : 0;
    const double fRT = vRT ? tmp - sqrt(distXH + distYL) : 0;
    const double fLB = vLB ? tmp - sqrt(distXL + distYH) : 0;
    const double fRB = vRB ? tmp - sqrt(distXH + distYH) : 0;
    const double sum = fLT + fRT + fLB + fRB;

    return ((pLT * fLT +  pRT * fRT + pLB * fLB + pRB * fRB) / sum) + 0.5;
}


void ImageCloudConverter::createLookup()
{
    const double fx = 1.0 / cameraMatrixRegistered.at<double>(0, 0);
    const double fy = 1.0 / cameraMatrixRegistered.at<double>(1, 1);
    const double cx = cameraMatrixRegistered.at<double>(0, 2);
    const double cy = cameraMatrixRegistered.at<double>(1, 2);
    double *it;

    lookupY = cv::Mat(1, sizeRegistered.height, CV_64F);
    it = lookupY.ptr<double>();
    for(size_t r = 0; r < (size_t)sizeRegistered.height; ++r, ++it)
    {
        *it = (r - cy) * fy;

    }

    lookupX = cv::Mat(1, sizeRegistered.width, CV_64F);
    it = lookupX.ptr<double>();
    for(size_t c = 0; c < (size_t)sizeRegistered.width; ++c, ++it)
    {
        *it = (c - cx) * fx;
    }

   /* for (int i = 50; i <60 ; ++i) {
        std::cout<<"ly="<<lookupY.at<double>(0, i)<<std::endl;

        std::cout<<"lx="<<lookupX.at<double>(0, i)<<std::endl;
    }*/
}

void ImageCloudConverter::setYamlPath(string path) {
    path_yaml=path;
}

void ImageCloudConverter::setInputImages(Mat &rawColor, Mat &rawDepth) {
    color= rawColor.clone();
    cv::flip(color, color, 1);

    rawDepth.convertTo(depth, CV_16U, 1, depthShift); //depthshift
    cv::flip(depth, depth, 1);
}

void ImageCloudConverter::rectifyColor() {
    if (mode==HD)
        cv::remap(color, color_rect, mapX_colorHD, mapY_colorHD, cv::INTER_AREA);
        else if(mode==QHD) {
         cv::remap(color, color_qhd_rect, mapX_colorQHD,mapY_colorQHD, cv::INTER_AREA);
    }

}

void ImageCloudConverter::setMode(int mode) {
    this->mode=mode;
}

void ImageCloudConverter::initRectify() {


    const int mapType = CV_16SC2;
    if (mode==HD)
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, Size(1920, 1080),
                                mapType, mapX_colorHD,mapY_colorHD);
    if (mode== QHD)
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrix_qhd, Size(1920/2, 1080/2),
                                mapType, mapX_colorQHD,mapY_colorQHD);

    cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixRegistered, sizeRegistered, CV_32FC1, mapX_depth, mapY_depth);
}
