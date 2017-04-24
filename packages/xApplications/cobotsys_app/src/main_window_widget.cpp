//
// Created by 潘绪洋 on 17-4-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_file_finder.h>
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
    m_uiCtrlActionGroup->setEnabled(false);
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

    m_objectEnumerator = new SharedObjectEnumerator;
    m_objectEnumerator->setParent(this);
    m_objectEnumerator->setWindowFlags(Qt::Dialog);
    m_objectEnumerator->initWidgetList();
    connect(ui.action_New, &QAction::triggered, this, &MainWindow::onNewWidget);
}

void MainWindow::onStart() {
}

void MainWindow::onPause() {
}

void MainWindow::onStop() {
}

void MainWindow::onViewLogger() {
    if (m_loggerWidget) {
        if (m_loggerWidget->isHidden()) {
            m_loggerWidget->show();
        } else {
            m_loggerWidget->hide();
        }
    }
}

void MainWindow::createMainWidget() {
}

QString getJSONConfig(QWidget* parent = nullptr) {
    auto robotConfig = QFileDialog::getOpenFileName(parent,
                                                    QCoreApplication::instance()->tr("Get Widget Config JSON file ..."),
                                                    QString(FileFinder::getPreDefPath().c_str()),
                                                    QCoreApplication::instance()->tr("JSON files (*.JSON *.json)"));
    return robotConfig;
}

void MainWindow::onNewWidget() {
    if (m_objectEnumerator->exec() == QDialog::Accepted) {
        QString factory = m_objectEnumerator->getFactory();
        QString typen = m_objectEnumerator->getType();

        auto dynObject = GlobalObjectFactory::instance()->createObject(factory, typen);
        auto widget = std::dynamic_pointer_cast<QWidget>(dynObject);
        if (widget) {
            auto robotConfig = getJSONConfig(this);
            if (robotConfig.isEmpty()) {
                setWidget(dynObject);
            } else {
                if (dynObject->setup(robotConfig)) {
                    setWidget(dynObject);
                }
            }
        }
    }
}

void MainWindow::setWidget(std::shared_ptr<AbstractObject>& shared_obj) {
    auto widget = std::dynamic_pointer_cast<QWidget>(shared_obj);
    if (widget) {
        setCentralWidget(widget.get());
        m_widgetObject = shared_obj;
    }

    auto ctrl = std::dynamic_pointer_cast<AbstractController>(shared_obj);
    if (ctrl) {
        m_uiCtrlActionGroup->setEnabled(true);
    } else {
        m_uiCtrlActionGroup->setEnabled(false);
    }
}
