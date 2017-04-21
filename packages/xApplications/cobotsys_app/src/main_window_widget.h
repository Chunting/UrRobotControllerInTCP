//
// Created by 潘绪洋 on 17-4-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_MAIN_WINDOW_WIDGET_H
#define COBOTSYS_MAIN_WINDOW_WIDGET_H

#include <QMainWindow>
#include <cobotsys_abstract_controller.h>
#include "ui_main_window_widget.h"
#include "basic_logger_widget.h"
#include <QActionGroup>

using namespace cobotsys;

class MainWindow : public QMainWindow {
Q_OBJECT
public:
    MainWindow(QWidget* parent = nullptr, Qt::WindowFlags flags = Qt::WindowFlags());
    virtual ~MainWindow();


protected:
    void setupUi();

    void onStart();
    void onPause();
    void onStop();

    void onViewLogger();

protected:
    std::shared_ptr<AbstractObject> m_widgetObject;
    Ui::MainWindow ui;
    QActionGroup* m_uiCtrlActionGroup;
    BasicLoggerWidget* m_basicLoggerWidget;
};


#endif //COBOTSYS_MAIN_WINDOW_WIDGET_H
