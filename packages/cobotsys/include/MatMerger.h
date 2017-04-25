//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_MATMERGER_H
#define PROJECT_MATMERGER_H

#include <vector>
#include <iostream>
#include <iomanip>
#include <cv.h>
#include <QImage>
#include <QDebug>
#include <QMutex>
#include <QPainter>
#include <cobotsys_logger.h>

QImage cv_mat_to_qimage(const cv::Mat &mat);


class MatLayoutData {
public:
    MatLayoutData() : x(0), y(0){
        width = 0;
        height = 0;
    }

    MatLayoutData(const std::string &n, int xx, int yy, int dw = 0, int dh = 0)
            : name(n), x(xx), y(yy), width(dw), height(dh){}

public:
    std::string name;
    int x;
    int y;
    int width;
    int height;

    bool isValidData() const{ return !name.empty(); }

    bool isScaled() const{ return width > 0 && height > 0; }

    QPoint point() const{ return QPoint(x, y); }
};

std::ostream &operator<<(std::ostream &oss, const MatLayoutData &layoutData);

void write(cv::FileStorage &fs, const std::string &, const MatLayoutData &data);
void read(const cv::FileNode &node, MatLayoutData &data, const MatLayoutData &default_value = MatLayoutData());

class MatMerger {
public:
    MatMerger();
    MatMerger(const MatMerger &r);
    ~MatMerger();

    bool loadMatLayout(const std::string &path);
    void saveMatLayout(const std::string &path);

    MatMerger &operator=(const MatMerger &r);

    void write(cv::FileStorage &fs) const;
    void read(const cv::FileNode &fn);

    std::vector<MatLayoutData> &getLayoutArray(){ return layout_array; }


    void updateMat(const std::string &name, const cv::Mat &mat);

    void draw(QPainter &painter);

    void clear();

    void imshow(const std::string &name);
protected:

protected: // 类copy的时候，需要copy的数据
    std::vector<MatLayoutData> layout_array;

protected:
    std::map<std::string, cv::Mat> cached_mat;
    QMutex inner_lock;
};

void write(cv::FileStorage &fs, const std::string &, const MatMerger &data);
void read(const cv::FileNode &node, MatMerger &data, const MatMerger &default_value = MatMerger());


#endif //PROJECT_MATMERGER_H
