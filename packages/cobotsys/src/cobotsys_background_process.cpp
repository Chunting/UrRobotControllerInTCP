//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QRegularExpression>
#include <cobotsys.h>
#include <signal.h>
#include <cobotsys_file_finder.h>
#include "cobotsys_background_process.h"
#include <cobotsys.h>


namespace cobotsys {

namespace ros_con_special {
#define NO_COLOR        "\033[0m"
#define FG_BLACK        "\033[30m"
#define FG_RED          "\033[31m"
#define FG_GREEN        "\033[32m"
#define FG_YELLOW       "\033[33m"
#define FG_BLUE         "\033[34m"
#define FG_MAGENTA      "\033[35m"
#define FG_CYAN         "\033[36m"

const char* none = "";

void replace_con_pattern(QByteArray& ba, const char* s_part, const char* e_part) {
    int pos_cur = 0;
    int pos_idx = 0;
    int pos_m;
    while (ba.size()) {
        pos_idx = ba.indexOf(s_part, pos_cur);
        if (pos_idx < 0)
            break;

        pos_m = ba.indexOf(e_part, pos_idx);
        if (pos_m < 0)
            break;
        ba.remove(pos_idx, pos_m - pos_idx + 1);
    }
}


QByteArray& replace_con_chars(QByteArray& ba) {
    replace_con_pattern(ba, "\033[", "m");
    replace_con_pattern(ba, "\033]", ";");
    ba.replace('\07', none);
//    int pos_cur = 0;
//    int pos_idx = 0;
//    int pos_m;
//    while (ba.size()) {
//        pos_idx = ba.indexOf("\033[", pos_cur);
//        if (pos_idx < 0)
//            break;
//
//        pos_m = ba.indexOf("m", pos_idx);
//        if (pos_m < 0)
//            break;
//        ba.remove(pos_idx, pos_m - pos_idx + 1);
//    }

//    ba.replace(NO_COLOR, none);
//    ba.replace(FG_BLACK, none);
//    ba.replace(FG_RED, none);
//    ba.replace(FG_GREEN, none);
//    ba.replace(FG_YELLOW, none);
//    ba.replace(FG_BLUE, none);
//    ba.replace(FG_MAGENTA, none);
//    ba.replace(FG_CYAN, none);
    return ba;
}
}

BackgroundProcess::BackgroundProcess(QObject* parent) : QObject(parent) {
    _process = nullptr;
}

BackgroundProcess::~BackgroundProcess() {
    kill();
}

void BackgroundProcess::run(const ProcessRunSettings& runSettings) {
    if (_process) {
        kill();
    }

    auto file = cobotsys::FileFinder::find(runSettings.getPath().toLocal8Bit().constData());
    if (file.empty()) {
        COBOT_LOG.error() << "Can't file file: " << runSettings.getPath();
        return;
    }

    auto qfile = QString::fromLocal8Bit(file.c_str());
    _run_settings = runSettings;
    _no_print = !runSettings.showConsolePrint();

    // Create process and connect process-io
    _process = new QProcess(this);
    connect(_process, &QProcess::readyReadStandardError, this, &BackgroundProcess::readStandardError);
    connect(_process, &QProcess::readyReadStandardOutput, this, &BackgroundProcess::readStandardOutput);
    connect(_process, static_cast<void (QProcess::*)(int, QProcess::ExitStatus)>(&QProcess::finished), this,
            &BackgroundProcess::processFinish);

    // Setup run environment
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    for (const auto& iter : runSettings.getEnvMap()) {
        env.insert(iter.first, iter.second);
    }
    _process->setProcessEnvironment(env);

    // Setup ros_arg
    QStringList ros_arg_list = gen_ros_internal_args(runSettings.getRosParamMap());

    // Start process
    QString command;
    switch (runSettings.getTaskProcessType()) {
    case TaskProcessType::Program:command = qfile + " " + runSettings.getCommandLine();
        COBOT_LOG.notice() << "Program: " << command.toLocal8Bit().constData();
        _process->start(command);
        break;
    case TaskProcessType::BashScript:_process->start("bash", QStringList() << "-c" << qfile);
        break;
    case TaskProcessType::RosProgram:_process->start(qfile, ros_arg_list);
        break;
    }
}

void BackgroundProcess::kill() {

#ifdef Q_OS_LINUX
    kill_process_childs(_process->pid(), 0, [=](int pid, int ppid) {
        COBOT_LOG.notice() << "CHILD KILL PID: " << std::setw(6) << pid << ", parent: " << ppid;
        ::kill(pid, SIGINT);
        ::kill(pid, SIGKILL);
    });
#endif

    _process->terminate();
    _process->kill();
    _process->waitForFinished(-1);
}

void process_out_to_log(QByteArray& prev, QByteArray& cur) {
    ros_con_special::replace_con_chars(cur);

    if (prev.size())
        prev += cur;
    else
        prev = cur;

    qt_ba_to_cobot_log(prev);
}

void BackgroundProcess::readStandardError() {
    if (_no_print) return;

    auto ba = _process->readAllStandardError();
    process_out_to_log(_std_err, ba);
}

void BackgroundProcess::readStandardOutput() {
    if (_no_print) return;

    auto ba = _process->readAllStandardOutput();
    process_out_to_log(_std_err, ba);
}

void BackgroundProcess::processFinish(int exit, QProcess::ExitStatus exitStatus) {
    emit processFinished(exit);
}


//
}