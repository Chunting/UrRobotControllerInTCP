//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtCore/QJsonParseError>
#include <QJsonObject>
#include <QJsonDocument>
#include <cobotsys_file_finder.h>
#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include "include/extra2.h"
#include <QDebug>

void qt_ba_to_cobot_log(QByteArray& ba){
    int search_index = 0;
    int pos = 0;

    while (search_index < ba.size()) {
        pos = ba.indexOf('\n', search_index);
        if (pos > 0) {
            auto line_ba = ba.mid(search_index, pos - search_index);
            search_index = pos + 1;
            COBOT_LOG.println(line_ba.constData());
        } else {
            ba = ba.mid(search_index);
            return;
        }
    }
    ba.clear();
}


QString get_subprocess_pid_list(int pid){
    QProcess killer;
    QStringList params;
    params << "--ppid";
    params << QString::number(pid);
    params << "-o";
    params << "pid";
    params << "--noheaders";
    killer.start("/bin/ps", params, QIODevice::ReadOnly);
    if (killer.waitForStarted(-1)) {
        if (killer.waitForFinished(-1)) {
            QByteArray temp = killer.readAllStandardOutput();
            return QString::fromLocal8Bit(temp).trimmed();
        }
    }
    return QString();
}

void kill_process_childs(int pid, int ppid, std::function<void(int, int)> killMethod){
    auto str = get_subprocess_pid_list(pid);
    QStringList list = str.split(QRegularExpression("\\s+"), QString::SkipEmptyParts);

    if (list.size()) {
        std::set<int> pids;
        for (const auto& iter : list) {
            pids.insert(iter.trimmed().toInt());
        }

        for (const auto& cid : pids) {
            // kill all child of cid
            kill_process_childs(cid, pid, killMethod);

            // kill cid
            if (killMethod)
                killMethod(cid, pid);
        }
    }
}

QStringList gen_ros_internal_args(const std::map<QString, QString>& arg_map){
    QStringList ros_arg_list;
    for (const auto& arg : arg_map) {
        QString ros_arg = "_";
        ros_arg += arg.first;
        ros_arg += ":=";
        ros_arg += arg.second;
        ros_arg_list << ros_arg;
    }
    return ros_arg_list;
}

bool loadJson(QJsonObject& obj, const std::string& baseName){
    QJsonParseError jsonParseError;

    auto fullPath = cobotsys::FileFinder::find(baseName);

    if (fullPath.empty())
        return false;

    QFile qFile(fullPath.c_str());
    if (qFile.open(QIODevice::ReadOnly)) {
        QTextStream qTextStream(&qFile);
        auto qss = qTextStream.readAll();
        auto jDoc = QJsonDocument::fromJson(qss.toUtf8(), &jsonParseError);
        if (jsonParseError.error == QJsonParseError::NoError) {
            obj = jDoc.object();
            return true;
        }
    }
    return false;
}


QImage matToQImage(const cv::Mat& mat){
    // 8-bits unsigned, NO. OF CHANNELS = 1
    if (mat.type() == CV_8UC1) {
        QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
        // Set the color table (used to translate colour indexes to qRgb values)
        image.setColorCount(256);
        for (int i = 0; i < 256; i++) {
            image.setColor(i, qRgb(i, i, i));
        }
        // Copy input Mat
        uchar* pSrc = mat.data;
        for (int row = 0; row < mat.rows; row++) {
            uchar* pDest = image.scanLine(row);
            memcpy(pDest, pSrc, mat.cols);
            pSrc += mat.step;
        }
        return image;
    }
        // 8-bits unsigned, NO. OF CHANNELS = 3
    else if (mat.type() == CV_8UC3) {
        // Copy input Mat
        const uchar* pSrc = (const uchar*) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        return image.rgbSwapped();
    } else if (mat.type() == CV_8UC4) {
//        qDebug() << "CV_8UC4";
        // Copy input Mat
        const uchar* pSrc = (const uchar*) mat.data;
        // Create QImage with same dimensions as input Mat
        QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
        return image.copy();
    } else {
        qDebug() << "ERROR: Mat could not be converted to QImage.";
        return QImage();
    }
}

bool loadJson(QJsonObject& obj, const QString& baseName){
    return loadJson(obj, std::string(baseName.toLocal8Bit().constData()));
}

