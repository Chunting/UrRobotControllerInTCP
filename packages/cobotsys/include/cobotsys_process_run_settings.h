//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_PROCESSRUNSETTINGS_H
#define PROJECT_PROCESSRUNSETTINGS_H

#include <ostream>
#include <iomanip>
#include <cv.h>
#include <map>
#include <QFileInfo>
#include <QFile>
#include <string>

namespace cobotsys {


enum class TaskProcessType {
    Program,
    BashScript,
    RosProgram
};

std::ostream &operator<<(std::ostream &oss, TaskProcessType t);


class ProcessRunSettings {
public:
    ProcessRunSettings();
    ~ProcessRunSettings();

    void write(cv::FileStorage &fs) const;
    void read(const cv::FileNode &fn);

    const std::map<QString, QString> &getEnvMap() const;
    const std::map<QString, QString> &getRosParamMap() const;
    const QString &getPath() const;
    const QString &getCommandLine() const;
    bool showConsolePrint() const;
    TaskProcessType getTaskProcessType() const;

    void debugPrint() const;
protected:
    std::map<QString, QString> _custom_env_vars;
    std::map<QString, QString> _custom_ros_vars;
    QString _command_line;
    QString _path;
    bool _show_console_print;
    TaskProcessType _task_process_type;
};

class BackgroundTaskSettings {
public:
    BackgroundTaskSettings();
    ~BackgroundTaskSettings();

    void write(cv::FileStorage &fs) const;
    void read(const cv::FileNode &fn);

    const std::vector<ProcessRunSettings> &getTaskSettings() const;

    void debugPrint(const std::string &taskName = std::string());
protected:
    std::vector<ProcessRunSettings> _process_settings;
};


template<class T>
void write(cv::FileStorage &fs, const std::string &, const T &data){
    data.write(fs);
}

template<class T>
void read(const cv::FileNode &node, T &data, const T &def_val = T()){
    if (node.empty())
        data = def_val;
    else
        data.read(node);
}

//
}

#endif //PROJECT_PROCESSRUNSETTINGS_H
