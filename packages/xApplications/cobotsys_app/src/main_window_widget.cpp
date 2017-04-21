//
// Created by 潘绪洋 on 17-4-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "main_window_widget.h"

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags) : QMainWindow(parent, flags) {
    setupUi();
}

MainWindow::~MainWindow() {
}

void MainWindow::setupUi() {
    ui.setupUi(this);

    m_uiCtrlActionGroup = new QActionGroup(this);
    m_uiCtrlActionGroup->addAction(ui.action_Start);
    m_uiCtrlActionGroup->addAction(ui.action_Pause);
    m_uiCtrlActionGroup->addAction(ui.action_Stop);
    ui.toolBar->addActions(m_uiCtrlActionGroup->actions());
    connect(ui.action_Start, &QAction::triggered, this, &MainWindow::onStart);
    connect(ui.action_Start, &QAction::triggered, this, &MainWindow::onPause);
    connect(ui.action_Start, &QAction::triggered, this, &MainWindow::onStop);

    ui.toolBar->addSeparator();
    ui.toolBar->addAction(ui.action_Show_Logger);
    connect(ui.action_Show_Logger, &QAction::triggered, this, &MainWindow::onViewLogger);

    /// Logger
    if (GlobalObjectFactory::instance()) {
        auto obj = GlobalObjectFactory::instance()->createObject("SimpleUiFactory, Ver 1.0", "BasicLoggerWidget");
        m_loggerWidget = std::dynamic_pointer_cast<QWidget>(obj);
        m_loggerWidget->setWindowFlags(Qt::Window);
    }
}

void MainWindow::onStart() {
}

void MainWindow::onPause() {
}

void MainWindow::onStop() {
}

void MainWindow::onViewLogger() {
    if (m_loggerWidget) {
        if (ui.action_Show_Logger->isChecked()) {
            m_loggerWidget->show();
        } else {
            m_loggerWidget->hide();
        }
    }
}
