//
// Created by vimjian on 17-4-14.
//

#include "detector.h"

#define COLOR_THR 30
#define HSV_THR 6
#define RECT_MINSIZE 15
#define RECT_MAXSIZE RECT_MINSIZE*1.7


Detector::Detector()
{

}

void Detector::FindGraspPoint(Mat &colorMat, Point &graspPoint)
{

    Mat gray_image, canny_image, roi_image;
    cvtColor(colorMat, gray_image, COLOR_RGB2GRAY);
    //  imshow("grayImage", gray_image);
    roi_image = gray_image(Range(300, 1000), Range(300, 1800));
    imshow("roi", roi_image);
    drawHist(roi_image, 50, 0, 0, false);
    blur(roi_image, roi_image, Size(9, 9));
    //imshow("blur",roi_image);
    // Canny(roi_image, canny_image, 30, 60, 3);
    // imshow("canny", canny_image);
    threshold(roi_image, roi_image, 5, 200, CV_THRESH_OTSU);
    //  imshow("ostu",roi_image);
    vector<vector<Point> > contours;
    findContours(roi_image, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    Mat monoColor(roi_image.size(), CV_8UC3, Scalar(255, 255, 255));
    drawContours(monoColor, contours, -1, Scalar(21, 22, 2), 3, 4);
    imshow(",", monoColor);

    cout << endl << "after sort" << endl;

    int64 t0 = getTickCount();

    //将contours的大小按照降序排序
    for (int j = 0; j < contours.size() - 1; ++j)
    {
        for (int i = 0; i < contours.size() - j - 1; ++i)
        {
            if (contours.at(i).size() < contours.at(i + 1).size())
            {
                vector<Point> tmp = contours.at(i);
                contours.at(i) = contours.at(i + 1);
                contours.at(i + 1) = tmp;
            }
        }
    }

    for (int i = 0; i < 3; ++i)
    {
        RotatedRect a = minAreaRect(contours.at(i));
        rectangle(monoColor, a.boundingRect(), Scalar(12, 300, 0));

    }
    imshow("3", monoColor);
    int64 t1 = getTickCount();
    cout << "time " << (t1 - t0) * 1000 / getTickFrequency();


}

bool compareGraspRectArea(const graspRect &a, const graspRect &b)
{
    return a.area > b.area;
}

void Detector::FindGraspRegion_ColorLabelHSV(Mat &colorMat)
{

    if (colorMat.cols == 1920)
        imageScale = 1.9;
    else if (colorMat.cols == 960)
        imageScale = 1.1;
    else
        imageScale = 1;

    cout << "imageScale " << imageScale << endl;

    Mat gray_image, canny_image, roi_image, blur_image;
    vector<Mat> channels;

    /*
     * 裁剪图像
     */
    roi_image = colorMat;//(Range(300, 1000), Range(300, 1800));
    Mat roi_color = colorMat;//(Range(300, 1000), Range(300, 1800));
    //imshow("roi", roi_image);

    /*
     * 分离通道
     */
    //   split(colorMat, channels);

    // imshow("1",channels.at(0));
    // imshow("2",channels.at(1));
    // imshow("3", channels.at(2));
    //blur(roi_image, blur_image, Size(5, 5));
    Mat monoColor;

    colorDet.setTargetHSV(Vec3b(134, 0, 0));//商标的颜色 216, 165, 101/ 245, 214, 152
    // colorDet.setTargetColor(Vec3b( 182,142,176));//商标的颜色 216, 165, 101/ 245, 214, 152
    colorDet.setColorDistanceThreshold(HSV_THR);
    monoColor = colorDet.processHSV(roi_image);
    imshow("binMat", monoColor);


    //闭运算
    Mat element = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(monoColor, monoColor, MORPH_CLOSE, element);
    imshow("morph", monoColor);

    /*
     * 二值化
     */
    //blur(roi_image, roi_image, Size(3, 3));
    //imshow("blur",roi_image);
    // Canny(roi_image, canny_image, 30, 60, 3);
    // imshow("canny", canny_image);
    //  threshold(roi_image, roi_image, 5, 200, CV_THRESH_OTSU);
    //threshold(roi_image, roi_image, 240, 200, CV_THRESH_BINARY);


    /*
     * 寻找轮廓
     */
    //  imshow("ostu",roi_image);
    vector<vector<Point> > contours;
    findContours(monoColor, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
    cout << "Find contours number:" << contours.size() << endl;
    Mat contoursMat(monoColor.size(), CV_8UC1, Scalar(255, 255, 255));
    drawContours(contoursMat, contours, -1, Scalar(21, 22, 2), 3, 4);
    imshow("contours", contoursMat);


    int64 t0 = getTickCount();

    /*
     * 寻找包围轮廓
     */

    for (int i = 0; i < contours.size(); ++i)
    {
        RotatedRect rect = minAreaRect(contours.at(i));
        double rectDia = sqrt(rect.size.width * rect.size.width + rect.size.height * rect.size.height);
      // cout << rectDia << endl;
        if (RECT_MINSIZE * imageScale <= rectDia && rectDia <= RECT_MAXSIZE * imageScale)
        {
            if (contourArea(contours.at(i)) > 100 && contourArea(contours.at(i)) < 1000)
            {
                rectangle(roi_color, rect.boundingRect(), Scalar(0, 255, 0), 2);
                graspRect grasp_rect;
                Point2i p;
                p.x = rect.center.x;
                p.y = rect.center.y;
                grasp_rect.rectCenter = p;
                grasp_rect.area = rect.size.height * rect.size.width;
                candidateRects.push_back(grasp_rect);
                // cout<<"candidate point "<<i<<": "<<p.x<<" "<<p.y<<endl;
            }
        }

    }
    //imshow("region", monoColor);
    sort(candidateRects.begin(), candidateRects.end(), compareGraspRectArea);
    for (int k = 0; k < candidateRects.size(); ++k)
    {
        candidatePoints.push_back(candidateRects.at(k).rectCenter);
    }

    int64 t1 = getTickCount();
    //cout << "time " << (t1 - t0) * 1000 / getTickFrequency() << endl;

#ifdef debug
    /*
     * 拟合圆
     */
    vector<Vec3f> circles;
    HoughCircles(monoColor, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 20, 10, 30);//
    cout<<"circles num: "<<circles.size()<<endl;


    //依次在图中绘制出圆
    for (size_t i = 0; i < circles.size(); i++)
    {
        //参数定义
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆轮廓
        circle(roi_color, center, radius, Scalar(0, 0,255), 2, 8, 0);
    }
#endif
    //显示效果图
    imshow("效果图", roi_color);

}

void Detector::FindGraspRegion_ColorLabel(Mat &colorMat)
{

    if (colorMat.cols == 1920)
        imageScale = 1.5;
    else if (colorMat.cols == 960)
        imageScale = 1.1;
    else
        imageScale = 1;

   // cout << "imageScale " << imageScale << endl;

    Mat gray_image, canny_image, roi_image, blur_image;
    vector<Mat> channels;

    /*
     * 裁剪图像
     */
    roi_image = colorMat(Range(100, 1000), Range(100, 1200));

    /*
     * 分离通道
     */
    //   split(colorMat, channels);

    // imshow("1",channels.at(0));
    // imshow("2",channels.at(1));
    // imshow("3", channels.at(2));
    //blur(roi_image, blur_image, Size(5, 5));
    Mat monoColor;

    colorDet.setTargetColor(182, 142, 176);//商标的颜色 182, 142, 176
    colorDet.setColorDistanceThreshold(COLOR_THR);
    monoColor = colorDet.process(roi_image);
    // imshow("binMat", monoColor);


    //闭运算
    Mat element1 = getStructuringElement(MORPH_RECT, Size(9, 9));
    morphologyEx(monoColor, monoColor, MORPH_CLOSE, element1);
    Mat element2 = getStructuringElement(MORPH_CROSS, Size(3, 3));
    morphologyEx(monoColor, monoColor, MORPH_DILATE, element2);
    // imshow("morph", monoColor);
    /*
     * 二值化
     */
    //blur(roi_image, roi_image, Size(3, 3));
    //imshow("blur",roi_image);
    // Canny(roi_image, canny_image, 30, 60, 3);
    // imshow("canny", canny_image);
    //  threshold(roi_image, roi_image, 5, 200, CV_THRESH_OTSU);
    //threshold(roi_image, roi_image, 240, 200, CV_THRESH_BINARY);


    /*
     * 寻找轮廓
     */
    //  imshow("ostu",roi_image);
    vector<vector<Point>> contours;
    findContours(monoColor, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  //  cout << "Find contours number:" << contours.size() << endl;
    Mat contoursMat(monoColor.size(), CV_8UC1, Scalar(255, 255, 255));
    drawContours(contoursMat, contours, -1, Scalar(21, 22, 2), 3, 4);
    // imshow("contours", contoursMat);


    int64 t0 = getTickCount();

    /*
     * 寻找包围轮廓
     */
    for (int i = 0; i < contours.size(); ++i)
    {
        RotatedRect rect = minAreaRect(contours.at(i));
        double rectDia = sqrt(rect.size.width * rect.size.width + rect.size.height * rect.size.height);
      //  cout << rectDia << endl;
        if (RECT_MINSIZE * imageScale <= rectDia && rectDia <= RECT_MAXSIZE * imageScale)
        {
            if (rect.size.height / rect.size.width * 1.0 < 3 && rect.size.height / rect.size.width * 1.0 > 0.33)
            {
                rectangle(m_colorImageShow, rect.boundingRect(), Scalar(50, 250, 0), 2, 4);
                graspRect grasp_rect;
                Point2i p;
                p.x = rect.center.x;
                p.y = rect.center.y;
                grasp_rect.rectCenter = p;
                grasp_rect.area = rect.size.height * rect.size.width;
                candidateRects.push_back(grasp_rect);
                // cout<<"candidate point "<<i<<": "<<p.x<<" "<<p.y<<endl;
            }
        }

    }
    //imshow("region", monoColor);
    int64 t1 = getTickCount();
 //   cout << "time " << (t1 - t0) * 1000 / getTickFrequency() << endl;

#ifdef debug
    /*
     * 拟合圆
     */
    vector<Vec3f> circles;
    HoughCircles(monoColor, circles, CV_HOUGH_GRADIENT, 2, 20, 100, 20, 10, 30);//
    cout<<"circles num: "<<circles.size()<<endl;


    //依次在图中绘制出圆
    for (size_t i = 0; i < circles.size(); i++)
    {
        //参数定义
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //绘制圆轮廓
        circle(roi_color, center, radius, Scalar(0, 0,255), 2, 8, 0);
    }
#endif

}

void Detector::FindGraspRegion(Mat &colorMat, Mat &depthMat)
{

    Mat gray_image, roi_image;
    vector<Mat> channels;

    // imshow("3", depthMat);

    split(depthMat, channels);

    //imshow("1",channels.at(0));
    //imshow("2",channels.at(1));
    //imshow("3", channels.at(2));
    roi_image = channels.at(0);

    //equalizeHist(roi_image,roi_image);
    //imshow("4",roi_image);
  //  cout << roi_image.type() << endl;
    drawHist(roi_image, 256, 0, 0, false);


}

void Detector::drawHist(Mat &sourceMat, int binsNum, float minRange, float maxRange, bool messageOut)
{
    Mat disHist;

    float hranges[] = {minRange, maxRange};
    const float *ranges[] = {hranges};
    int channels = 0;

    calcHist(&sourceMat, 1, &channels, Mat(), disHist, 1, &binsNum, ranges, true, false);

    if (messageOut)
    {
//        for (int j = 0; j < binsNum; ++j)
//        {
//        //    cout << "bin" << j << ":" << disHist.at<float>(j) << endl;
//        }
    }

    double minValue = 0;
    double maxValue = 0;
    minMaxLoc(disHist, &minValue, &maxValue, 0, 0);
    int scale = 300 / binsNum;
    int histHeight = 256;
    Mat distImg(histHeight, binsNum * scale + 1, CV_8U, Scalar(0));
    //draw with rectangle
    for (int i = 0; i < binsNum; ++i)
    {
        float binValue = disHist.at<float>(i);
        int binHeight = saturate_cast<int>(binValue * histHeight / maxValue);

        Point p0(i * scale, histHeight - 1);
        Point p1((i + 1) * scale, histHeight - binHeight);
        rectangle(distImg, p0, p1, Scalar(255));
    }

    imshow("直方图", distImg);
}

void Detector::FindGraspRegion_barCode_brightness(Mat &colorMat)
{
    /*
     * 定义
     */
    Mat roi_image, blur_image;
    vector<Mat> channels;

    /*
     * 裁剪图像
     */
    roi_image = colorMat(Range(100, 1000), Range(100, 1200));
    Mat roi_color = colorMat(Range(100, 1000), Range(100, 1200)).clone();//(Range(300, 1000), Range(300, 1800));

    /*
     * 转换为灰度图,二值图,滤波
     */
    //cv::blur(roi_image, roi_image, Size(3, 3), Point(-1, -1), BORDER_DEFAULT);
    Mat grayMat, binMat;
    cvtColor(roi_image, grayMat, COLOR_BGR2GRAY);
    //imshow("grayImage", grayMat);
    //threshold(grayMat, binMat, 5, 200, CV_THRESH_OTSU);
    //adaptiveThreshold(grayMat,binMat,250,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,3,5);
    threshold(grayMat, binMat, 150, 250, CV_THRESH_BINARY);
    //去掉部分噪声点
    medianBlur(binMat, binMat, 3);
    imshow("binImage", binMat);

    Mat edge;
    Laplacian(grayMat, edge, CV_8U, 3, 1);
    threshold(edge, edge, 200, 250, CV_THRESH_BINARY);
    imshow("edgeImage", edge);


    /*
     * 形态学计算
     */
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    Mat element1 = getStructuringElement(MORPH_RECT, Size(9, 9));
    Mat element2 = getStructuringElement(MORPH_CROSS, Size(3, 3));
    // morphologyEx(binMat,binMat,MORPH_CLOSE,element);
    morphologyEx(binMat, binMat, MORPH_OPEN, element1);
    morphologyEx(binMat, binMat, MORPH_CLOSE, element2);

    //imshow("morph",binMat);

    /*
     *寻找轮廓
     */
    vector<vector<Point> > contours;
    findContours(binMat, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
   // cout << "Contours number :" << contours.size() << endl;

    /*
     * visualization
     */
    //绘制轮廓
    Mat monoColor(roi_image.size(), CV_8UC3, Scalar(255, 255, 255));
    drawContours(monoColor, contours, -1, Scalar(21, 25, 2), 3, 4);
    imshow("contours", monoColor);

    regionNum = 0;
    //绘制方形
    for (int i = 0; i < contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(contours.at(i));

        Point2f vertex[4];
        box.points(vertex);
        if (contourArea(contours.at(i)) > 350 && contourArea(contours.at(i)) < 1000)
        {
            if (box.boundingRect().x < 0 || box.boundingRect().y < 0 ||
                (box.boundingRect().x + box.boundingRect().width) >= colorMat.cols ||
                (box.boundingRect().y + box.boundingRect().height) >= colorMat.rows
                    )
                continue; // 超出了图片边界
            Mat region = grayMat(box.boundingRect());
            if (!validateRegion_barCode(region))
                continue;//图形区域不是条形码
            graspRect grasp_rect;
            Point2i p;
            p.x = box.center.x;
            p.y = box.center.y;
            grasp_rect.rectCenter = p;
            grasp_rect.area = box.size.height * box.size.width;
            candidateRects.push_back(grasp_rect);
            // cout<<"candidate point "<<i<<": "<<p.x<<" "<<p.y<<endl;

            for (int j = 0; j < 4; ++j)
            {
                line(roi_color, vertex[j], vertex[(j + 1) % 4], Scalar(12, 255, 0), 2);
            }
            //rectangle(roi_color, box.boundingRect(), Scalar(12, 255, 0)); //近似包围矩形
        }

    }

    sort(candidateRects.begin(), candidateRects.end(), compareGraspRectArea);
    for (int k = 0; k < candidateRects.size(); ++k)
    {
        candidatePoints.push_back(candidateRects.at(k).rectCenter);
    }

    cv::imshow("graspRegion", roi_color);
}

void
Detector::FindGraspRegion_barCode_connectRegion(Mat &colorMat) //http://blog.csdn.net/xdg_blog/article/details/52834581
{
    /*
     * 定义
     */
    Mat roi_image, blur_image;
    vector<Mat> channels;

    /*
     * 裁剪图像
     */
    roi_image = colorMat(Range(100, 1000), Range(100, 1200));

    /*
     * 转换为灰度图,均衡化,二值图
     */
    //cv::blur(roi_image, roi_image, Size(3, 3), Point(-1, -1), BORDER_DEFAULT);
    Mat grayMat, binMat;
    cvtColor(roi_image, grayMat, COLOR_BGR2GRAY);
    equalizeHist(grayMat, grayMat);
    //   imshow("edgeIma", grayMat);

/*    Mat edge;
    Laplacian(grayMat, edge, CV_8U, 3, 1);
    threshold(edge, edge, 200, 250, CV_THRESH_BINARY);
    imshow("edgeImage", edge);*/

    /*Mat sobel_0,sobel_0_abs,sobel_90,sobel_90_abs,sobel_abs;
    Sobel(grayMat,sobel_0,CV_16S,1,0,3,1,1,BORDER_DEFAULT);
    Sobel(grayMat,sobel_90,CV_16S,0,1,3,1,1,BORDER_DEFAULT);

    convertScaleAbs(sobel_0,sobel_0_abs);
    convertScaleAbs(sobel_90,sobel_90_abs);
    addWeighted(sobel_0_abs,0.5,sobel_90_abs,0.5,0,sobel_abs);
    imshow("sobel_0",sobel_0_abs);
    imshow("sobel_90",sobel_90_abs);
    imshow("sobel_abs",sobel_abs);


    Mat sobel_angle(sobel_abs.rows,sobel_abs.cols,CV_8UC1);
    for (int l = 0; l < sobel_abs.rows; ++l)
    {
        for (int i = 0; i < sobel_abs.cols; ++i)
        {
           if (sobel_abs.at<uchar>(l,i)>120)
            {


               uchar angle = 90 + 180*asin(0.5*sobel_90_abs.at<uchar>(l, i) / (float)sobel_abs.at<uchar>(l, i))/3.141592658;
                *//*cout<<"sobel_90_abs.at<uchar>(l, i):"<<(int)sobel_90_abs.at<uchar>(l, i)<<endl;
                cout<<"sobel_abs.at<uchar>(l, i):"<<(int)sobel_abs.at<uchar>(l, i)<<endl;
                cout<<"angle:"<<(int)angle<<endl;*//*
               sobel_angle.at<uchar>(l,i)=angle;

           }
           else
               sobel_angle.at<uchar>(l,i)=0;
        }

    }
    imshow("sobel_angle",sobel_angle);
    drawHist(sobel_angle, 30, 90, 180);*/

    //imshow("grayImage", grayMat);
    threshold(grayMat, binMat, 5, 250, CV_THRESH_OTSU);
    //adaptiveThreshold(grayMat,binMat,250,ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,3,5);
    //  threshold(grayMat, binMat, 180, 250, CV_THRESH_BINARY);
    //去掉部分噪声点
   // imshow("binImageRaw", binMat);
    medianBlur(binMat, binMat, 3);
    Mat element123 = getStructuringElement(MORPH_CROSS, Size(3, 3));
    morphologyEx(binMat, binMat, MORPH_CLOSE, element123);
   // imshow("binImage", binMat);

    /*
     * 寻找联通区域,填补孔洞
     */
    vector<vector<Point>> contoursRaw;
    findContours(binMat, contoursRaw, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  //  cout << endl << "contoursRaw number :" << contoursRaw.size() << endl;
    Mat fillMat = Mat(binMat.rows, binMat.cols, binMat.type(), Scalar(0, 0, 0));

/*    for (int l = 0; l < contoursRaw.size(); ++l) {
        if(contoursRaw[l].size()>maxArea)
        {
            maxArea=contoursRaw[l].size();
            maxAreaIndex=l;
        }
        cout<<"contours "<<l<<": "<<contoursRaw[l].size()<<endl;
    }
    cout<<"maxContours :"<<maxAreaIndex<<",Area "<<maxArea<<endl;
    */
    for (int i = 0; i < contoursRaw.size(); ++i)
    {
        if (contourArea(contoursRaw.at(i)) > 200 && contourArea(contoursRaw.at(i)) < 4000)
        {
            drawContours(fillMat, contoursRaw, i, Scalar(255, 255, 255), CV_FILLED);
        }
    }
  //  imshow("filled", fillMat);
    binMat = fillMat;


    /*
     * 形态学计算
     */
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    Mat element1 = getStructuringElement(MORPH_RECT, Size(11, 11));
    morphologyEx(binMat, binMat, MORPH_CLOSE, element);
    morphologyEx(binMat, binMat, MORPH_OPEN, element1);
    // imshow("morph",binMat);

    /*
     *寻找轮廓
     */
    vector<vector<Point>> contours;
    findContours(binMat, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  //  cout << "Contours number :" << contours.size() << endl;

    /*
     * visualization
     */
    //绘制轮廓
    Mat monoColor(roi_image.size(), CV_8UC3, Scalar(255, 255, 255));
    drawContours(monoColor, contours, -1, Scalar(21, 25, 2), 3, 4);
 //   imshow("contours", monoColor);

    regionNum = 0;
    //绘制方形
    for (int i = 0; i < contours.size(); ++i)
    {
        RotatedRect box = minAreaRect(contours.at(i));

        Point2f vertex[4];
        box.points(vertex);
        if (contourArea(contours.at(i)) < 300 || contourArea(contours.at(i)) > 800)
        {
            //       drawContours(m_colorImageShow, contours, i, Scalar(255, 0, 0), 2);
            continue;//区域大小不对
        }
        /*
         cout << endl << box.boundingRect().x << " " << box.boundingRect().y
              << " " << box.boundingRect().x + box.boundingRect().width
              << " " << box.boundingRect().y + box.boundingRect().height << endl;*/

        if (box.boundingRect().x < 0 || box.boundingRect().y < 0 ||
            (box.boundingRect().x + box.boundingRect().width) >= 1100 ||
            (box.boundingRect().y + box.boundingRect().height) >= 900
                )
        {
            //drawContours(m_colorImageShow, contours, i, Scalar(0, 0, 255), 1);
            continue; // 超出了图片边界
        }

        Mat region = grayMat(box.boundingRect());
        if (!validateRegion_barCode(region))
        {
            for (int j = 0; j < 4; ++j)
            {
                //line(m_colorImageShow, vertex[j], vertex[(j + 1) % 4], Scalar(0, 0, 255), 2);
            }
            continue;//图形区域不是条形码
        }

        graspRect grasp_rect;
        Point2i p;
        p.x = box.center.x;
        p.y = box.center.y;
        grasp_rect.rectCenter = p;
        grasp_rect.area = box.size.height * box.size.width;
        candidateRects.push_back(grasp_rect);
        // cout<<"candidate point "<<i<<": "<<p.x<<" "<<p.y<<endl;

        for (int j = 0; j < 4; ++j)
        {
            line(m_colorImageShow, vertex[j], vertex[(j + 1) % 4], Scalar(0, 255, 0), 2);
        }
        //rectangle(roi_color, box.boundingRect(), Scalar(12, 255, 0)); //近似包围矩形
    }

}

bool Detector::validateRegion_barCode(Mat &region)
{

    bool result = false;
    // imshow("region", region);

    //锐化
    Mat edge;
    Laplacian(region, edge, CV_8U, 3, 1);

    //Mat element =getStructuringElement(MORPH_CROSS,Size(3,3));
    //morphologyEx(edge,edge,MORPH_CLOSE,element);
    //imshow("edge2",edge);

    threshold(edge, edge, 35, 250, CV_THRESH_BINARY);

    /*  ostringstream os;
       os<<regionNum;
       imshow(os.str(),edge);*/

    regionNum++;
    //imshow("bin", edge);

    //Canny(edge,edge,24,72,3);
    //imshow("canny",edge);

    /*
     * 区域内对比度
     */

    double scaleLow = 0.28;
    double scaleHigh = 0.6;
    float hranges[] = {0, 255};
    const float *ranges[] = {hranges};
    int channels = 0;
    MatND disHist;
    int binsNum = 2;
    calcHist(&edge, 1, &channels, Mat(), disHist, 1, &binsNum, ranges, true, false);
    double scaleResult =
            (double) disHist.at<float>(0, 1) / (double) (disHist.at<float>(0, 0) + disHist.at<float>(0, 1)) * 1.0;
    if (scaleResult < scaleLow || scaleResult > scaleHigh)
        result = false;
    else
        result = true;
    /*
      int oneNum=0;
      int zeroNum=0;
      for (int i = 0; i < edge.rows; ++i) {
          uchar *data=edge.ptr<uchar>(i);
          for (int j = 0; j < edge.cols; ++j) {
              if (data[j] !=0)
                  oneNum++;
              else
                  zeroNum++;

          }

      }
      double scaleResult=(double)oneNum/(oneNum+zeroNum)*1.0;
      if(scaleResult<scale)
          result=false;
      else
          result =true;
          */

    /*
     * 梯度计算
     */
/*
  Mat sobel_0,sobel_0_abs,sobel_90,sobel_90_abs,sobel_abs;
    Sobel(region,sobel_0,CV_16S,1,0,3,1,1,BORDER_DEFAULT);
    Sobel(region,sobel_90,CV_16S,0,1,3,1,1,BORDER_DEFAULT);

    convertScaleAbs(sobel_0,sobel_0_abs);
    convertScaleAbs(sobel_90,sobel_90_abs);
    addWeighted(sobel_0_abs,0.5,sobel_90_abs,0.5,0,sobel_abs);
    imshow("sobel_0",sobel_0_abs);
    imshow("sobel_90",sobel_90_abs);
    imshow("sobel_abs",sobel_abs);


    Mat sobel_angle(sobel_abs.rows,sobel_abs.cols,CV_8UC1);
    for (int l = 0; l < sobel_abs.rows; ++l)
    {
        for (int i = 0; i < sobel_abs.cols; ++i)
        {
           if (sobel_abs.at<uchar>(l,i)> 80)
            {
                if (sobel_0_abs.at<uchar>(l,i)!=0)
                {
                    uchar angle = 90 +
                                  180 * atan(0.5 * sobel_90_abs.at<uchar>(l, i) / (float) sobel_0_abs.at<uchar>(l, i)) /
                                  CV_PI;
                   */
/* cout << "sobel_90:" << (int) sobel_90_abs.at<uchar>(l, i) << endl;
                    cout << "sobel_0:" << (int) sobel_0_abs.at<uchar>(l, i) << endl;
                    cout << "angle:" << (int) angle << endl;*//*

                    sobel_angle.at<uchar>(l, i) = angle;
                }
                else
                {
                    sobel_angle.at<uchar>(l, i) = 90+90;
                }
           }
           else
               sobel_angle.at<uchar>(l,i)=0;
        }

    }
    imshow("sobel_angle",sobel_angle);
    drawHist(sobel_angle, 10,90, 180, false);
*/

   // cout << "region Num " << regionNum << " scale:" << scaleResult << " " << result << endl;

    return result;
}

void Detector::FindGraspRegion_combine_label_barCode(Mat &colorMat)
{
    candidatePoints.clear();
    candidateRects.clear();
    FindGraspRegion_ColorLabel(colorMat);
    FindGraspRegion_barCode_connectRegion(colorMat);
    sort(candidateRects.begin(), candidateRects.end(), compareGraspRectArea);
    for (int k = 0; k < candidateRects.size(); ++k)
    {
        candidatePoints.push_back(candidateRects.at(k).rectCenter);
    }
    cv::imshow("Grasp Region", m_colorImageShow); //show grasp region
    std::cout << "----------------------------------" ;
}

void Detector::SetColorImage(Mat &colorMat)
{
    m_colorImage = colorMat;
    m_colorImageShow = m_colorImage(Range(100, 1000), Range(100, 1200)).clone();;
}




