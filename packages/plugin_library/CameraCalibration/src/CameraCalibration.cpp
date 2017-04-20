//
// Created by eleven on 17-4-19.
//

#include "CameraCalibration.h"
#include "cobotsys_logger.h"
/*  
* 函数名称：CameraCalibration 
* 函数功能：类构造函数  
* 函数入口：   
* 输入参数：标定板横纵坐标角点数_board_sz, 相邻两次图像获取的时间间隔_board_dt（单位：秒）, 获取图像的总数_n_boards 
* 输出参数：无 
* 返 回 值：无 
* 其它说明：   
*/
CameraCalibration::CameraCalibration(const cv::Size boardSize, const string filename)
    : m_boardSize(boardSize)
    , m_outputFilename(filename)
{
    m_squareSize  = 1.f;
    m_aspectRatio = 1.f;
    m_nframes     = 10;
}

CameraCalibration::~CameraCalibration()
{

}

bool CameraCalibration::calibrateFromCamera(const cv::Mat& view, const Size imageSize)
{
//    namedWindow( "Image View", 1 );                                                        //[12]创建一个视频窗口

    Mat viewGray;
    vector<Point2f> pointbuf;
    cvtColor(view, viewGray, COLOR_BGR2GRAY);

    bool found = findChessboardCorners(view, m_boardSize, pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);


    if(found) {
        //【2】使用cornerSubPix()函数将角点坐标精确到亚像素级别
        cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1),
                     TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        //【4】使用drawChessboardCorners()绘制已经找出来的棋盘格角点
        drawChessboardCorners(view, m_boardSize, Mat(pointbuf), found);
        m_imagePoints.push_back(pointbuf);
        COBOT_LOG.info() << "findChessboardCorners success" << m_imagePoints.size();
    }
    else {
        COBOT_LOG.info() << "findChessboardCorners failed";
        return false;
    }


//    imshow("Image View", view);


    if(m_imagePoints.size() >= (unsigned)m_nframes ) //【5】开始进行摄像机标定
    {
        if(runAndSave(m_outputFilename,
                      m_imagePoints,
                      imageSize,
                      m_boardSize,
                      CHESSBOARD,
                      m_squareSize,
                      m_aspectRatio,
                      0,
                      m_cameraMatrix,
                      m_distCoeffs,
                      true,
                      false)) {
            COBOT_LOG.info() << "CameraCalibration success";
            return true;
        }
        else {
            COBOT_LOG.info() << "CameraCalibration failed";
            return false;
        }
    }
    else {
        return false;
    }

//    if(showUndistorted )                            //【6】进行图像校正
//    {
//        Mat view, rview, map1, map2;
//
//        initUndistortRectifyMap(cameraMatrix,
//                                distCoeffs,
//                                Mat(),
//                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
//                                imageSize,
//                                CV_16SC2,
//                                map1,
//                                map2);
//
//        for( i = 0; i < (int)imageList.size(); i++ )
//        {
//            view = imread(imageList[i], 1);
//            if(!view.data)
//                continue;
//            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
//            remap(view, rview, map1, map2, INTER_LINEAR);
//            imshow("Image View", rview);
//            int c = 0xff & waitKey();
//            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
//                break;
//        }
//    }

}/*  calibrateFromCamera()   */

/*
* 函数名称：calibrateFromCamera
* 函数功能：根据已获取的图像文件（.bmp格式），标定相机
* 函数入口：
* 输入参数：无
* 输出参数：无
* 返 回 值：是否标定成功，true表示成功，false表示失败
* 其它说明： 只接受.bmp格式的图片，且图片尺寸要相同，若要标定其他格式图片，请将本函数内的.bmp替换成.jpg
*            文件统一命名格式为 calib_N.bmp,其中N必须从0开始
*/
bool CameraCalibration::calibrateFromFile()
{
    return true;
} /* calibrateFromFile() */



double CameraCalibration::computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>&              rvecs,
        const vector<Mat>&              tvecs,
        const Mat&                      cameraMatrix,
        const Mat&                      distCoeffs,
        vector<float>&                  perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i              = 0;
    int totalPoints    = 0;
    double totalErr    = 0;                                                    //【1】单幅图像的平均误差
    double err         = 0;
    perViewErrors.resize(objectPoints.size());
    /*投影函数--对应OpenCv1.0版本中的cvProjectPoints2()函数---CCS--->ICS*/
    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]),                                    //【1】将要投影的摄像机坐标系下的三位点的坐标
                      rvecs[i],                                            //【2】平移矩阵
                      tvecs[i],                                            //【3】旋转矩阵
                      cameraMatrix,                                        //【4】摄像机内参数矩阵
                      distCoeffs,                                          //【5】摄像机畸变向量(径向畸变,切向畸变k1,k2,k3,p1,p2)
                      imagePoints2);                                       //【6】对于摄像机三维物理坐标框架下的位置,我们计算出来的该三维点在成像仪中的坐标(像素坐标)

        err              = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);//【7】两个数组对应元素差值平方的累加和
        int n            = (int)objectPoints[i].size();                        //【8】Vector向量的成员函数--resize(),size(),push_back(),pop_back()
        perViewErrors[i] = (float)std::sqrt(err*err/n);                        //【9】单个三维点的投影误差

        totalErr    += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);                                    //【10】摄像机投影的总体误差
}

void CameraCalibration::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType)
{
    corners.resize(0);                                               //【1】Vector向量的成员函数,重置Vector向量的长度为0

    switch(patternType)                                              //【2】判断标定板的类型
    {
        case CHESSBOARD:                                               //【3】棋盘格类型的标定板
        case CIRCLES_GRID:                                             //【4】圆心型风格的标定板
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float(j*squareSize),       //【5】将摄像机坐标系下的三维世界实际的标定板的物理坐标存储在corners这个vector向量容器类
                                              float(i*squareSize), 0));
            break;

        case ASYMMETRIC_CIRCLES_GRID:
            for( int i = 0; i < boardSize.height; i++ )
                for( int j = 0; j < boardSize.width; j++ )
                    corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                              float(i*squareSize), 0));
            break;

        default:
            COBOT_LOG.info() <<  "Unknown pattern type";
    }
}

bool CameraCalibration::runCalibration( vector<vector<Point2f> > imagePoints,
                            Size                     imageSize,
                            Size                     boardSize,
                            Pattern                  patternType,
                            float                    squareSize,
                            float                    aspectRatio,
                            int                      flags,
                            Mat&                     cameraMatrix,
                            Mat&                     distCoeffs,
                            vector<Mat>&             rvecs,
                            vector<Mat>&             tvecs,
                            vector<float>&           reprojErrs,
                            double&                  totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);                                         //【1】摄像机内部参数矩阵,创建一个3*3的单位矩阵

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);                                         //【2】摄像机的畸变系数向量,创建一个8*1的行向量

    vector<vector<Point3f> > objectPoints(1);

    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);    //【3】计算棋盘格角点世界坐标系下的三维物理坐标

    objectPoints.resize(imagePoints.size(),objectPoints[0]);                       //【4】对objectPoints的Vector容器进行扩容,并且扩充的内存空间用元素objectPoints[0]填充
    //【5】摄像机标定函数----计算摄像机的内部参数和外部参数
    double rms = calibrateCamera(objectPoints,      //【1】世界坐标系下*每张标定图片中的角点的总数k*图片的个数M---N*3矩阵(N=k*M)------物理坐标
                                 imagePoints,       //【2】imagePoints是一个N*2的矩阵,它由objectPoints所提供的所有点所对应点的像素坐标构成,
            //如果使用棋盘格进行标定,那么，这个变量简单的由M次调用cvFindChessboardCorners()的返回值构成
                                 imageSize,         //【3】imageSize是以像素衡量的图像的尺寸Size,图像点就是从该图像中提取的
                                 cameraMatrix,      //【4】摄像机内部参数矩阵--------定义了理想摄像机的摄像机行为
                                 distCoeffs,        //【5】畸变系数行向量5*1---8*1---更多的表征了摄像机的非理想行为
                                 rvecs,             //【6】rotation_vectors----------旋转矩阵
                                 tvecs);             //【7】tanslation_vectors--------平移矩阵
//                                 flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);          ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    COBOT_LOG.info() << "RMS error reported by calibrateCamera: " << rms;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);                  //【6】checkRange()函数---用于检查矩阵中的每一个元素是否在指定的一个数值区间之内

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,             //【7】完成摄像机标定后，对标定进行评价，计算重投影误差/摄像机标定误差
                                            rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);               //【8】函数的返回值是摄像机标定/投影的总体平均误差

    return ok;
}


void CameraCalibration::saveCameraParams( const string& filename,
                              Size imageSize, Size boardSize,
                              float squareSize, float aspectRatio, int flags,
                              const Mat& cameraMatrix, const Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs,
                              const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
                 flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                 flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                 flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                 flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() ) {
        Mat imagePtMat((int) imagePoints.size(), (int) imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int) imagePoints.size(); i++) {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

bool CameraCalibration::runAndSave(const string&                   outputFilename,
                                   const vector<vector<Point2f> >& imagePoints,
                                   Size                            imageSize,
                                   Size                            boardSize,
                                   Pattern                         patternType,
                                   float                           squareSize,
                                   float                           aspectRatio,
                                   int                             flags,
                                   Mat&                            cameraMatrix,
                                   Mat&                            distCoeffs,
                                   bool                            writeExtrinsics,
                                   bool                            writePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints,
                             imageSize,
                             boardSize,
                             patternType,
                             squareSize,
                             aspectRatio,
                             flags,
                             cameraMatrix,
                             distCoeffs,
                             rvecs,
                             tvecs,
                             reprojErrs,
                             totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename,
                          imageSize,
                          boardSize,
                          squareSize,
                          aspectRatio,
                          flags,
                          cameraMatrix,
                          distCoeffs,
                          writeExtrinsics ? rvecs : vector<Mat>(),
                          writeExtrinsics ? tvecs : vector<Mat>(),
                          writeExtrinsics ? reprojErrs : vector<float>(),
                          writePoints ? imagePoints : vector<vector<Point2f> >(),
                          totalAvgErr );
    return ok;
}