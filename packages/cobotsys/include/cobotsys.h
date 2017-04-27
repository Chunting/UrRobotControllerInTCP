//
// Created by 潘绪洋 on 17-2-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_H
#define PROJECT_COBOTSYS_H


#include <cobotsys_logger.h>
#include <string>
#include <vector>
#include <iomanip>
#include <iostream>
#include <functional>
#include <memory>
#include <ostream>

#include <QString>
#include <QJsonObject>
#include <QJsonDocument>
#include <QImage>
#include <QFont>
#include <QFontInfo>
#include <QDebug>

#include <opencv2/opencv.hpp>

using std::shared_ptr;
using std::make_shared;

namespace cobotsys {


void init_library(int argc, char** argv);


class cout_formater {
public:
    int section_width;
    int fixed_num_len;
    int fixed_pt_len;
    std::ostream& oss;

    cout_formater(std::ostream& o = std::cout) : oss(o) {
        section_width = 12;
        fixed_num_len = 10;
        fixed_pt_len = 3;
    }

    cout_formater& section(const std::string& title) {
        oss << "["
            << std::setw(section_width) << title
            << "]";
        return *this;
    }


    cout_formater& fixedNum(double n) {
        auto flags = oss.flags();
        oss.flags(std::ios_base::fixed);
        oss << std::setw(fixed_num_len) << std::setprecision(fixed_pt_len)
            << n;
        oss.flags(flags);
        return *this;
    }

    template<class T>
    cout_formater& printArray(const T* pData, uint32_t n) {
        for (uint32_t i = 0; i < n; i++) {
            fixedNum(pData[i]);
        }
        return *this;
    }

    cout_formater& newline() {
        oss << std::endl;
        return *this;
    }
};
}

std::ostream& operator<<(std::ostream& oss, const QString& str);
std::ostream& operator<<(std::ostream& oss, const QJsonObject& obj);
QFont getMonospaceFont();

#endif //PROJECT_COBOTSYS_H
