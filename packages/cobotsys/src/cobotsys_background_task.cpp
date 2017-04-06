//
// Created by 潘绪洋 on 17-2-20.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_file_finder.h>
#include <cobotsys_qt.h>
#include "cobotsys_background_task.h"

namespace cobotsys {

BackgroundTask::BackgroundTask(QObject* parent) : QObject(parent) {
}

BackgroundTask::~BackgroundTask() {
}

bool BackgroundTask::run(const BackgroundTaskSettings& settings) {
    // Check if setting file is empty.
    for (const auto& iter : settings.getTaskSettings()) {
        if (cobotsys::FileFinder::find(iter.getPath().toLocal8Bit().constData()).empty()) {
            COBOT_LOG.error() << "Can not find file: " << iter.getPath();
            return false;
        }
    }

    const auto& v_conf = settings.getTaskSettings();
    auto num_process = v_conf.size();

    _num_finished = 0;
    _process_list.clear();
    for (size_t i = 0; i < num_process; i++) {
        auto p = std::make_shared<BackgroundProcess>(nullptr);
        _process_list.push_back(p);
        connect(p.get(), &BackgroundProcess::processFinished, this, &BackgroundTask::onProcessFinish);
    }

    for (size_t i = 0; i < num_process; i++) {
        const auto& conf = v_conf[i];
        _process_list[i]->run(conf);
    }
    return true;
}

void BackgroundTask::onProcessFinish(int exitCode) {
    _num_finished++;

    if (_num_finished == _process_list.size()) {
        emit taskFinished();
    }
}

void BackgroundTask::stop() {
    for (auto& p : _process_list) {
        p->kill();
    }
    _process_list.clear();
}


//
}