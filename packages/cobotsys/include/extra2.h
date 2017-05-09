//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EXTRA_H
#define PROJECT_EXTRA_H

#include <set>
#include <map>
#include <cobotsys_logger.h>
#include <QByteArray>
#include <QString>
#include <QRegularExpression>
#include <QProcess>
#include <QStringList>
#include <QJsonObject>
#include <QJsonValue>
#include <QJsonArray>
#include <QImage>
#include <opencv2/opencv.hpp>
#include <cobotsys.h>
#include <qmath.h>
#include <iostream>
#include <iomanip>        // std::put_time
#include <thread>         // std::this_thread::sleep_until
#include <chrono>         // std::chrono::system_clock
#include <ctime>          // std::time_t, std::tm, std::localtime, std::mktime

void qt_ba_to_cobot_log(QByteArray& ba);
void kill_process_childs(int pid, int ppid, std::function<void(int, int)> killMethod);
QStringList gen_ros_internal_args(const std::map<QString, QString>& arg_map);


bool loadJson(QJsonObject& obj, const std::string& baseName);
bool loadJson(QJsonObject& obj, const QString& baseName);
QImage matToQImage(const cv::Mat& mat);

std::string simple_typeid_name(const char* pname);

#define INFO_DESTRUCTOR(_this) COBOT_LOG.info("Destructor") << simple_typeid_name(typeid(_this).name())


template<class T, class N, class M>
void range_limit(T& v, const N& min_, const M& max_) {
    if ((T) min_ < (T) max_) {
        if (v < (T) min_)
            v = (T) min_;
        if (v > (T) max_)
            v = (T) max_;
    }
}

std::vector<double> readRealArray(const QJsonValue& realArrayValue);

#endif //PROJECT_EXTRA_H
