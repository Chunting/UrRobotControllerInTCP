//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_BINPICKINGGUI_H
#define PROJECT_BINPICKINGGUI_H

#include "ui_BinpickingView.h"
#include <QWidget>
#include <QMenu>
#include <QAction>
#include <QPaintEvent>
#include <QTimer>
#include <QDateTime>
#include <QDebug>
#include <QPainter>
#include <cobotsys_process_run_settings.h>
#include <cobotsys_background_task.h>
#include "cobotsys.h"
#include "LoggerViewWidget.h"
#include "easy_cv_mat_reader.h"
#include "MatMerger.h"
#include "EasyGuiShowClient.h"
#include <cobotsys_background_process_master.h>

class BinpickingView : public QWidget {
Q_OBJECT
public:
    BinpickingView(QWidget *parent = nullptr);
    ~BinpickingView();


    void reloadLayoutConfig();
    void clearLoggerWindowText();

    void actionStart();
    void actionStop();
    void actionCalibration();
    void actionClear();

    void actionCloseAllViewMatWindow();


    void showDebugUi();

Q_SIGNALS:

    void commandTriggered(const QString &command);

protected:
    void loadConfig();
    void loadRunScript();

    void stopAll();

    void genViewMatMenu();
    void updateViewMatMenu();
    void enableMatView();

    void timerStart();
    void timerStop();
    void timerInfoUpdate();

protected:
    void setupLoggerUi();
    void onTaskFinish();
protected:
    virtual void paintEvent(QPaintEvent *event);
protected:
    Ui::BinpickingView ui;

    LoggerViewWidget *_logger_widget;
    EasyGuiShowClient *_easy_gui_show_client;

    cobotsys::BackgroundTaskSettings _settings_binpicking;
    cobotsys::BackgroundTaskSettings _settings_calibration;

    cobotsys::BackgroundTask *_task_binpicking;
    cobotsys::BackgroundTask *_task_calibration;

    qint64 _time_start_count;
    bool _time_enabled;
    QTimer *_time_updater;

    QMenu *_view_mat_menu;
    QStringList _view_mat_names_old;
    QPixmap _logo;

    cobotsys::BackgroundProcessMaster *_master;
};


#endif //PROJECT_BINPICKINGGUI_H
