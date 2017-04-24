//
// Created by 潘绪洋 on 17-4-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_MAIN_WINDOW_WIDGET_H
#define COBOTSYS_MAIN_WINDOW_WIDGET_H

#include <QMainWindow>
#include <QFileDialog>
#include <cobotsys_abstract_controller.h>
#include <cobotsys_abstract_widget.h>
#include <cobotsys_global_object_factory.h>
#include "ui_main_window_widget.h"
#include "SharedObjectEnumerator.h"
#include <QActionGroup>
#include <QCloseEvent>

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

    void createMainWidget();
    void onNewWidget();

    void setWidget(std::shared_ptr<AbstractObject>& shared_obj);


protected:
    virtual void closeEvent(QCloseEvent* event);

protected:
    std::shared_ptr<AbstractObject> m_widgetObject;
    Ui::MainWindow ui;
    QActionGroup* m_uiCtrlActionGroup;
    std::shared_ptr<QWidget> m_loggerWidget;

    SharedObjectEnumerator* m_objectEnumerator;
};


#endif //COBOTSYS_MAIN_WINDOW_WIDGET_H
