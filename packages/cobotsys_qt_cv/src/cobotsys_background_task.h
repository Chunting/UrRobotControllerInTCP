//
// Created by 潘绪洋 on 17-2-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_BACKGROUNDTASK_H
#define PROJECT_BACKGROUNDTASK_H

#include <memory>
#include <QObject>
#include "cobotsys_process_run_settings.h"
#include "cobotsys_background_process.h"

namespace cobotsys {

class BackgroundTask : public QObject {
Q_OBJECT
public:
    BackgroundTask(QObject *parent = nullptr);
    ~BackgroundTask();


    void run(const BackgroundTaskSettings &settings);
    void stop();

Q_SIGNALS:
    void taskFinished();
protected:
    void onProcessFinish(int exitCode);
protected:
    typedef std::shared_ptr<BackgroundProcess> process_t;
    std::vector<process_t> _process_list;
    size_t _num_finished;
};

//
}

#endif //PROJECT_BACKGROUNDTASK_H
