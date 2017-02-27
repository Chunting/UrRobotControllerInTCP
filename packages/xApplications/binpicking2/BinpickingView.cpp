//
// Created by 潘绪洋 on 17-2-9.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_file_finder.h>
#include "BinpickingView.h"

#define LAYOUT_XML_FILE "binpicking_image_layout.xml"
#define BINPICKING_FILE "binpicking_action_config.xml"

BinpickingView::BinpickingView(QWidget *parent) :
        QWidget(parent){
    _logger_widget = nullptr;
    _easy_gui_show_client = nullptr;

    // Init timer
    _time_start_count = QDateTime::currentMSecsSinceEpoch();
    _time_enabled = false;
    _time_updater = new QTimer(this);
    _time_updater->setInterval(25);
    connect(_time_updater, &QTimer::timeout, this, &BinpickingView::timerInfoUpdate);

    _view_mat_menu = nullptr;


    _task_binpicking = new cobotsys::BackgroundTask(this);
    _task_calibration = new cobotsys::BackgroundTask(this);
    connect(_task_binpicking, &cobotsys::BackgroundTask::taskFinished, this, &BinpickingView::onTaskFinish);
    connect(_task_calibration, &cobotsys::BackgroundTask::taskFinished, this, &BinpickingView::onTaskFinish);

    _master = new cobotsys::BackgroundProcessMaster(this);
    _master->getMaster().lanuchMaster();


    ui.setupUi(this);
    ui.debugUI->hide();

    _logo = QPixmap::fromImage(QImage(":/icons/resources/cobot_logo_icon.png"));

    connect(ui.btnStart, &QPushButton::released, this, &BinpickingView::actionStart);
    connect(ui.btnStop, &QPushButton::released, this, &BinpickingView::actionStop);
    connect(ui.btnCalibration, &QPushButton::released, this, &BinpickingView::actionCalibration);
    connect(ui.btnClear, &QPushButton::released, this, &BinpickingView::actionClear);
    connect(ui.btnViewMatClear, &QPushButton::released, this, &BinpickingView::actionCloseAllViewMatWindow);

    setupLoggerUi();
    loadConfig();
    loadRunScript();

    genViewMatMenu();
}

BinpickingView::~BinpickingView(){
}


void BinpickingView::paintEvent(QPaintEvent *event){
    QPainter painter(this);

    if (_easy_gui_show_client) {
        _easy_gui_show_client->draw(painter);
    }

    painter.drawPixmap(6, 6, _logo);

    QWidget::paintEvent(event);
}


void BinpickingView::stopAll(){
    if (_task_binpicking) _task_binpicking->stop();
    if (_task_calibration) _task_calibration->stop();
    _easy_gui_show_client->clearClientMat();
}

void BinpickingView::setupLoggerUi(){
    auto loggerlayout = new QVBoxLayout;
    auto loggerWidget = new LoggerViewWidget(this);

    loggerlayout->setContentsMargins(0, 0, 0, 0);
    loggerlayout->addWidget(loggerWidget);
    ui.loggerFrame->setLayout(loggerlayout);
    loggerWidget->setOpacity();
    loggerWidget->bindCurrentProcessLogger();

    _logger_widget = loggerWidget;

    _easy_gui_show_client = new EasyGuiShowClient(this);
    _easy_gui_show_client->initShowClient();
    connect(_easy_gui_show_client, &EasyGuiShowClient::clientDataUpdated, [=](){ this->update(); });
}

void BinpickingView::reloadLayoutConfig(){
    auto layout_xml_config = cobotsys::FileFinder::find(LAYOUT_XML_FILE);
    COBOT_LOG.notice() << "Binpicking Layout: " << layout_xml_config;
}


void BinpickingView::clearLoggerWindowText(){
    if (_logger_widget) {
        _logger_widget->clear();
    }
}

void BinpickingView::actionStart(){
    if (_task_binpicking->run(_settings_binpicking)) {
        ui.btnViewMat->setEnabled(true);

        timerStart();

        ui.btnStart->setEnabled(false);
        ui.btnCalibration->setEnabled(false);
    }
}

void BinpickingView::actionStop(){
    stopAll();
    update();
}

void BinpickingView::actionCalibration(){
    if (_task_calibration->run(_settings_calibration)) {
        timerStart();

        ui.btnStart->setEnabled(false);
        ui.btnCalibration->setEnabled(false);
    }
}

void BinpickingView::loadConfig(){
    auto layout_xml_config = cobotsys::FileFinder::find(LAYOUT_XML_FILE);
    COBOT_LOG.notice() << "Binpicking Layout: " << layout_xml_config;
    cv::FileStorage fs(layout_xml_config, cv::FileStorage::READ);
    if (fs.isOpened()) {
        _easy_gui_show_client->loadLayoutConfig(fs);
    }
}

void BinpickingView::loadRunScript(){
    auto run_script_path = cobotsys::FileFinder::find(BINPICKING_FILE);
    COBOT_LOG.notice() << "Binpicking Layout: " << run_script_path;
    cv::FileStorage fs(run_script_path, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["action_binpicking"] >> _settings_binpicking;
        fs["action_calibration"] >> _settings_calibration;

        _settings_binpicking.debugPrint("binpicking");
        _settings_calibration.debugPrint("calibration");
    }
}

void BinpickingView::onTaskFinish(){
    ui.btnStart->setEnabled(true);
    ui.btnCalibration->setEnabled(true);
    timerStop();
}

void BinpickingView::actionClear(){
    clearLoggerWindowText();
}

void BinpickingView::genViewMatMenu(){
    if (_view_mat_menu == nullptr) {
        _view_mat_menu = new QMenu(this);
    }

    ui.btnViewMat->setMenu(_view_mat_menu);
    updateViewMatMenu();
}

void BinpickingView::enableMatView(){
    auto action = dynamic_cast<QAction *>(sender());
    if (action) {
        auto im_name = action->text();
        _easy_gui_show_client->showWithOpenCvApi(im_name);
    }
}

bool __list_name_equal(const QStringList &a, const QStringList &b){
    if (a.size() == b.size()) {
        for (int i = 0; i < a.size(); i++) {
            if (a.at(i) != b.at(i))
                return false;
        }
        return true;
    }
    return false;
}

void BinpickingView::updateViewMatMenu(){
    auto names = _easy_gui_show_client->getConnectedMatNames();
    names.sort();
    _view_mat_names_old.sort();
    if (__list_name_equal(names, _view_mat_names_old)) {
        return;
    }
    _view_mat_names_old = names;

    _view_mat_menu->clear();
    for (auto &name : names) {
        auto action = new QAction;
        action->setText(name);
        action->setCheckable(true);
        action->setChecked(_easy_gui_show_client->getCvMatViewStatus(name));
        connect(action, &QAction::triggered, this, &BinpickingView::enableMatView);
        _view_mat_menu->addAction(action);
    }
}

void BinpickingView::actionCloseAllViewMatWindow(){
    _easy_gui_show_client->destoryAllCvShowWindow();
    _view_mat_names_old.clear();
    updateViewMatMenu();
}

void BinpickingView::timerStart(){
    _time_updater->start();
    _time_start_count = QDateTime::currentMSecsSinceEpoch();
    _time_enabled = true;
}

void BinpickingView::timerStop(){
    _time_updater->stop();
}

void BinpickingView::timerInfoUpdate(){
    if (_time_enabled) {
        auto tm_used = QDateTime::currentMSecsSinceEpoch() - _time_start_count;
        ui.labelRunTime->setText(QString("%1 second ").arg(tm_used / 1000.0, 0, 'f', 2));

        updateViewMatMenu();
    }
}

void BinpickingView::showDebugUi(){
    ui.debugUI->show();
}

