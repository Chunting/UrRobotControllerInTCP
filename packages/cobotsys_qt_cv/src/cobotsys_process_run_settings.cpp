//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_logger.h>
#include "cobotsys_process_run_settings.h"

namespace cobotsys {


void key_value_write(const QString &name, const std::map<QString, QString> &m, cv::FileStorage &fs){
    fs << name.toLocal8Bit().constData() << "[";
    for (const auto &iter : m) {
        fs << "{";
        fs << "key" << iter.first.toLocal8Bit().constData();
        fs << "value" << iter.second.toLocal8Bit().constData();
        fs << "}";
    }
    fs << "]";
}

void key_value_read(const QString &name, std::map<QString, QString> &m, const cv::FileNode &fn){
    auto fnIter = fn[name.toLocal8Bit().constData()];
    if (fnIter.type() == cv::FileNode::SEQ) {
        m.clear();
        for (const auto &n : fnIter) {
            std::string key = (n["key"]);
            std::string value = (n["value"]);
            if (key.size()) {
                m[QString::fromLocal8Bit(key.c_str())] = QString::fromLocal8Bit(value.c_str());
            }
        }
    }
}


ProcessRunSettings::ProcessRunSettings(){
    _task_process_type = TaskProcessType::Program;
}

ProcessRunSettings::~ProcessRunSettings(){
}

void ProcessRunSettings::write(cv::FileStorage &fs) const{
    fs << "{";
    key_value_write("ENV", _custom_env_vars, fs);
    key_value_write("ROS_PARAM", _custom_ros_vars, fs);
    fs << "path" << _path.toLocal8Bit().constData();
    fs << "is_bash_script" << (int) _task_process_type;
    fs << "show_console_print" << (int) _show_console_print;
    fs << "command_line" << std::string(_command_line.toLocal8Bit().constData());
    fs << "}";
}

void ProcessRunSettings::read(const cv::FileNode &fn){
    key_value_read("ENV", _custom_env_vars, fn);
    key_value_read("ROS_PARAM", _custom_ros_vars, fn);

    std::string path = fn["path"];
    int task_process_type = fn["task_process_type"];
    int show_console_print = fn["show_console_print"];
    std::string cmdline = fn["command_line"];

    _path = QString::fromLocal8Bit(path.c_str());
    _task_process_type = (TaskProcessType) task_process_type;
    _show_console_print = (show_console_print != 0);
    _command_line = QString::fromLocal8Bit(cmdline.c_str());
}

const std::map<QString, QString> &ProcessRunSettings::getEnvMap() const{
    return _custom_env_vars;
}

const std::map<QString, QString> &ProcessRunSettings::getRosParamMap() const{
    return _custom_ros_vars;
}

const QString &ProcessRunSettings::getPath() const{
    return _path;
}

bool ProcessRunSettings::showConsolePrint() const{
    return _show_console_print;
}


void BackgroundTaskSettings::write(cv::FileStorage &fs) const{
    fs << "Task" << "{" << "[";
    for (const auto &iter : _process_settings) {
        fs << iter;
    }
    fs << "]" << "}";
}

void BackgroundTaskSettings::read(const cv::FileNode &fn){
    auto fnIter = fn["Task"];
    if (fnIter.type() == cv::FileNode::SEQ) {
        _process_settings.clear();
        for (const auto &n : fnIter) {
            ProcessRunSettings tmp_settings;
            n >> tmp_settings;
            _process_settings.push_back(tmp_settings);
        }
    }
}

const std::vector<ProcessRunSettings> &BackgroundTaskSettings::getTaskSettings() const{
    return _process_settings;
}

BackgroundTaskSettings::BackgroundTaskSettings(){
}

BackgroundTaskSettings::~BackgroundTaskSettings(){
}


void BackgroundTaskSettings::debugPrint(const std::string &taskName){
    COBOT_LOG.notice() << "[" << setw(20) << taskName << "] total process num: " << _process_settings.size();
    for (const auto &conf : _process_settings) {
        conf.debugPrint();
    }
}

static std::ostream &operator<<(std::ostream &oss, const QString &s){
    oss << s.toLocal8Bit().constData();
    return oss;
}

void ProcessRunSettings::debugPrint() const{
    COBOT_LOG.notice() << std::left << setw(10) << "Program" << ": " << _path.toLocal8Bit().constData();
    COBOT_LOG.message() << std::left << setw(10) << "TaskType" << ": " << _task_process_type;
    COBOT_LOG.message() << std::left << setw(10) << "Print" << ": " << std::boolalpha << _show_console_print;

    if (_command_line.size())
        COBOT_LOG.message() << std::left << setw(10) << "CmdLine" << ": " << _command_line;

    for (const auto &var : _custom_env_vars) {
        COBOT_LOG.message() << std::left << setw(10) << "ENV" << ": "
                            << setw(16) << var.first << " = " << var.second;
    }

    for (const auto &var : _custom_ros_vars) {
        COBOT_LOG.message() << std::left << setw(10) << "ROS_PARAM" << ": "
                            << setw(16) << var.first << " = " << var.second;
    }
}

TaskProcessType ProcessRunSettings::getTaskProcessType() const{
    return _task_process_type;
}

const QString &ProcessRunSettings::getCommandLine() const{
    return _command_line;
}


std::ostream &operator<<(std::ostream &oss, TaskProcessType t){
    switch (t) {
        case TaskProcessType::Program:
            oss << std::setw(12) << "Program";
            break;
        case TaskProcessType::BashScript:
            oss << std::setw(12) << "BashScript";
            break;
        case TaskProcessType::RosProgram:
            oss << std::setw(12) << "RosProgram";
            break;
    }
    return oss;
}

//
}