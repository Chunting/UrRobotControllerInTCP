//
// Created by eleven on 17-4-19.
//

#ifndef COBOTSYS_CAMERACALIBRATION_H
#define COBOTSYS_CAMERACALIBRATION_H

#include <opencv2/opencv.hpp>

using namespace cv;

enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

class CameraCalibration {
public:
    CameraCalibration(const cv::Size boardSize, const string filename);
    ~CameraCalibration();

public:
    /*
    * 函数名称：calibrateFromCamera
    * 函数功能：直接从相机实时获取标定板图像，用于标定
    * 函数入口：
    * 输入参数：五
    * 输出参数：无
    * 返 回 值：是否标定成功，true表示成功，false表示失败
    * 其它说明：
    */
    bool calibrateFromCamera(const cv::Mat& view, const cv::Size imageSize);
    bool calibrateFromFile();

private:
    /****************************************************************************************************************************
    函数功能:
           完成摄像机标定之后，对摄像机标定的结果进行评价，计算重投影误差/摄像机标定误差
    函数参数:
           1---const vector<vector<Point3f> >& objectPoints----常引用类型----三维坐标系下的物理坐标点
           2---const vector<vector<Point2f> >& imagePoints ----图像坐标系下的二维坐标点----成像仪
           3---const Mat& cameraMatrix-------------------------摄像机内参矩阵
           4---const Mat& distCoeffs---------------------------摄像机畸变向量
           5---const vector<Mat>&              rvecs-----------旋转矩阵
           6---const vector<Mat>&              tvecs-----------平移矩阵
    函数返回值:
           误差率
    ****************************************************************************************************************************/
    double computeReprojectionErrors(
            const vector<vector<Point3f> >& objectPoints,
            const vector<vector<Point2f> >& imagePoints,
            const vector<Mat>&              rvecs,
            const vector<Mat>&              tvecs,
            const Mat&                      cameraMatrix,
            const Mat&                      distCoeffs,
            vector<float>&                  perViewErrors );

    /****************************************************************************************************************************
    函数原型:
           static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
    函数功能:
           计算棋棋盘格----世界坐标系下----真实的物理三维坐标点的坐标
    函数参数:
           1---Size boardSize-------------棋盘格的尺寸Size
           2---float squareSize-----------棋盘格角点之间的距离Size
           3---vector<Point3f>& corners---用来存储棋盘格角点的三维坐标
           4---Pattern patternType--------标定板的类型
    函数返回值:
           void
    ****************************************************************************************************************************/
    void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD);

    /****************************************************************************************************************************
    函数原型:
           static bool runCalibration(略）
    函数功能:
           摄像机标定模块最核心的模块--------摄像机标定函数
    函数参数:
           vector<vector<Point2f> > imagePoints,          //【1】输入控制变量---WCS下三维物理点的3D坐标数组,自己手动推导出来的三位坐标结果
           Size                     imageSize,            //【2】输入控制变量---ICS坐标系下,根据findChessboardCorners()函数计算出来的标定图片上角点的坐标
           Size                     boardSize,            //【3】输入控制变量---棋盘格的Size---棋盘格的横纵【内角点】个数
           Pattern                  patternType,          //【4】输入控制变量---方块形棋盘格,原型型棋盘格,标定板模式
           float                    squareSize,           //【5】输入控制变量---棋盘格角点之间的距离/圆心型标定板圆心之间的距离
           float                    aspectRatio,          //【6】输入控制变量---长宽比
           int                      flags,                //【7】输入控制变量---标志位
           Mat&                     cameraMatrix,         //【1】待求解输出控制变量 ---摄像机内参数矩阵
           Mat&                     distCoeffs,           //【2】待求解输出控制变量 ---摄像机畸变系数矩阵
           vector<Mat>&             rvecs,                //【3】待求解输出控制变量---旋转矩阵
           vector<Mat>&             tvecs,                //【4】待求解输出控制变量---畸变向量(k1,k2,k3,p1,p2)
           vector<float>&           reprojErrs,           //【5】待求解输出控制变量---单幅图片/单个投影点的---投影误差/摄像机标定误差
           double&                  totalAvgErr           //【6】待求解输出控制变量---摄像机标定的总体平均误差/投影平均误差
    函数返回值:
           摄像机标定是否成功-----求解摄像机的内外参数矩阵是否成功
    ****************************************************************************************************************************/
    bool runCalibration( vector<vector<Point2f> > imagePoints,
                                Size                     imageSize,
                                Size                     boardSize,
                                Pattern

                                patternType,
                                float                    squareSize,
                                float                    aspectRatio,
                                int                      flags,
                                Mat&                     cameraMatrix,
                                Mat&                     distCoeffs,
                                vector<Mat>&             rvecs,
                                vector<Mat>&             tvecs,
                                vector<float>&           reprojErrs,
                                double&                  totalAvgErr);

    void saveCameraParams( const string& filename,
                                  Size imageSize, Size boardSize,
                                  float squareSize, float aspectRatio, int flags,
                                  const Mat& cameraMatrix, const Mat& distCoeffs,
                                  const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                  const vector<float>& reprojErrs,
                                  const vector<vector<Point2f> >& imagePoints,
                                  double totalAvgErr );

    bool runAndSave(const string&                   outputFilename,
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
                           bool                            writePoints );

    Size m_boardSize;                                          //[1]标定板的Size

    float m_squareSize;                                        //[3]棋盘格角点之间的距离
    float m_aspectRatio;                                       //[4]长宽比

    Mat   m_cameraMatrix;                                      //[5]摄像机的内参数矩阵
    Mat   m_distCoeffs;                                        //[6]摄像机的畸变系数向量

    int   m_nframes;
    vector<vector<Point2f> > m_imagePoints;

    string m_outputFilename;                                  //[7]输出的Xml文件名
};




#endif //COBOTSYS_CAMERACALIBRATION_H
