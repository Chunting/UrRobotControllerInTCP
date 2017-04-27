//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <highgui.h>
#include "MatMerger.h"
#include <opencv2/opencv.hpp>

QImage cv_mat_to_qimage(const cv::Mat &mat){
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (mat.type() == CV_8UC1) {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for (int i = 0; i < 256; i++) {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar *pSrc = mat.data;
        for (int row = 0; row < mat.rows; row++) {
            uchar *pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
        // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (mat.type() == CV_8UC3) {
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    } else if (mat.type() == CV_8UC4) {
//        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar *pSrc = (const uchar *) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    } else {
        cv::Mat newMat;
        mat.convertTo(newMat, CV_8UC1);
        return cv_mat_to_qimage(newMat);

        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

void write(cv::FileStorage &fs, const std::string &, const MatLayoutData &data){
    fs << "{"
       << "name" << data.name
       << "x" << data.x
       << "y" << data.y
       << "width" << data.width
       << "height" << data.height
       << "}";
}

void read(const cv::FileNode &node, MatLayoutData &data, const MatLayoutData &default_value){
    if (node.empty()) {
        data = default_value;
    } else {
        data.name = (std::string) node["name"];
        data.x = (int) node["x"];
        data.y = (int) node["y"];
        data.width = (int) node["width"];
        data.height = (int) node["height"];
    }
}

void write(cv::FileStorage &fs, const std::string &, const MatMerger &data){
    data.write(fs);
}

void read(const cv::FileNode &node, MatMerger &data, const MatMerger &default_value){
    if (node.empty())
        data = default_value;
    else
        data.read(node);
}

std::ostream &operator<<(std::ostream &oss, const MatLayoutData &layoutData){
    oss << std::setw(16) << layoutData.name << ", "
        << std::setw(7) << layoutData.x
        << ", "
        << std::setw(7) << layoutData.y
        << ",\t" << layoutData.width
        << ",\t" << layoutData.height;
    return oss;
}

MatMerger::MatMerger(){
}

MatMerger::~MatMerger(){
}

#define MAT_MERGER_NODE "MatMerger"

bool MatMerger::loadMatLayout(const std::string &path){
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (fs.isOpened()) {
        layout_array.clear();
        fs[MAT_MERGER_NODE] >> (*this);
        return true;
    }
    return false;
}

void MatMerger::saveMatLayout(const std::string &path){
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << MAT_MERGER_NODE << (*this);
    }
}

void MatMerger::write(cv::FileStorage &fs) const{
    fs << "{";
    fs << "layouts" << "[";
    for (auto &dat : layout_array) {
        fs << dat;
    }
    fs << "]";
    fs << "}";
}


void MatMerger::read(const cv::FileNode &fn){
    auto fsIter = fn["layouts"];
    if (fsIter.type() == cv::FileNode::SEQ) {
        layout_array.clear();
        for (const auto &iter : fsIter) {
            MatLayoutData matLay;
            iter >> matLay;
            layout_array.push_back(matLay);

            COBOT_LOG.notice() << matLay;
        }
    }
}

MatMerger::MatMerger(const MatMerger &r) : layout_array(r.layout_array){
}

void MatMerger::updateMat(const std::string &name, const cv::Mat &mat){
    inner_lock.lock();
    cached_mat[name] = mat;
    inner_lock.unlock();
//    COBOT_LOG.notice() << "image updated, " << name;
}

MatMerger &MatMerger::operator=(const MatMerger &r){
    layout_array = r.layout_array;
    return *this;
}

void MatMerger::draw(QPainter &painter){
    std::map<std::string, QImage> images;
    std::map<std::string, MatLayoutData> cached_layout;

    inner_lock.lock();
    for (const auto &l : layout_array) {
        cached_layout[l.name] = l;
    }

    for (auto &iter : cached_mat) {
        if (cached_layout.find(iter.first) != cached_layout.end()) {
            if (iter.second.cols > 0 && iter.second.rows > 0) {
                images[iter.first] = cv_mat_to_qimage(iter.second);
            }
        }
    }
    inner_lock.unlock();

    // draw
    for (auto &iter : images) {
        const auto &layout_style = cached_layout[iter.first];
        if (layout_style.isScaled()) {
            painter.drawImage(QRect(layout_style.x, layout_style.y, layout_style.width, layout_style.height),
                              iter.second);
        } else {
            painter.drawImage(layout_style.point(), iter.second);
        }
    }
}

void MatMerger::clear(){
    inner_lock.lock();
    cached_mat.clear();
    inner_lock.unlock();
}

void MatMerger::imshow(const std::string &name){
    inner_lock.lock();
    auto &mat = cached_mat[name];
    if (mat.cols > 0 && mat.rows > 0)
        cv::imshow(name, cached_mat[name]);
    inner_lock.unlock();
}

