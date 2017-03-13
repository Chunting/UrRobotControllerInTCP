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
#include "extra2.h"

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
