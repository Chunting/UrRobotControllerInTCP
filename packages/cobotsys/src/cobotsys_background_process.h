//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_BACKGROUNDPROCESS_H
#define PROJECT_BACKGROUNDPROCESS_H

#include <QObject>
#include <QProcess>
#include "cobotsys_process_run_settings.h"
#include "extra2.h"


namespace cobotsys {

class BackgroundProcess : public QObject {
Q_OBJECT
public:
    BackgroundProcess(QObject *parent);
    ~BackgroundProcess();


    void run(const ProcessRunSettings &runSettings);
    void kill();

Q_SIGNALS:
    void processFinished(int exitCode);

protected:
    void readStandardError();
    void readStandardOutput();
    void processFinish(int exit, QProcess::ExitStatus exitStatus);
protected:
    QProcess *_process;
    ProcessRunSettings _run_settings;
    bool _no_print;
    QByteArray _std_err;
    QByteArray _std_out;
};

//
}

#endif //PROJECT_BACKGROUNDPROCESS_H
