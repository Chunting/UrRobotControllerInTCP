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


void qt_ba_to_cobot_log(QByteArray &ba);
void kill_process_childs(int pid, int ppid, std::function<void(int, int)> killMethod);
QStringList gen_ros_internal_args(const std::map<QString, QString> &arg_map);


#endif //PROJECT_EXTRA_H
