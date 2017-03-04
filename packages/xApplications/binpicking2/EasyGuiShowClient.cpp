//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "EasyGuiShowClient.h"

#define BINPICKING_EASY_GUI_LAYOUT "MatMerger"

EasyGuiShowClient::EasyGuiShowClient(QObject* parent)
        : QObject(parent), _client_ready(new bool(true)), _mat_merger(new MatMerger){

    _reload_timer = nullptr;
    _no_data_warning = true;
    connect(this, &EasyGuiShowClient::matReaderStatus, this, &EasyGuiShowClient::processMatReaderStatus);
}

EasyGuiShowClient::~EasyGuiShowClient(){
    *_client_ready = false;
    if (_reload_timer) {
        _reload_timer->stop();

        for (auto& iter : _mat_status) {
            if (iter.second.holder) {
                iter.second.holder->stopReader();
            }
        }
    }
}

void EasyGuiShowClient::draw(QPainter& painter){
    _mat_merger->draw(painter);
}

bool EasyGuiShowClient::loadLayoutConfig(const cv::FileStorage& fs){
    auto fn = fs[BINPICKING_EASY_GUI_LAYOUT];
    if (fn.empty())
        return false;

    _mat_merger->read(fn);
    return true;
}

void EasyGuiShowClient::reloadImageTargets(){
    auto names = _easy_cv_mat_reader.existMatNames();

    if (names.empty()) {
        if (_no_data_warning) {
            _no_data_warning = false;
            COBOT_LOG.warning() << "No cv_mat is exist!";
        }
        return;
    }
    _no_data_warning = true;

    std::vector<QString> qnames;
    for (auto name : names) {
        qnames.push_back(QString::fromLocal8Bit(name.c_str()));
    }
    dumpAndCompareNames(qnames);

    for (auto name : qnames) {
        auto s_name = name.toLocal8Bit().constData();

        if (_mat_status[name].isConnected)
            continue;

        _mat_status[name].holder = _easy_cv_mat_reader.lanuch(s_name,
                                                              [=](cobotsys::common::EasyCvMatReaderStatus status){
                                                                  emit matReaderStatus(name, (int) status);
                                                              });

        _mat_status[name].holder->setUpdateCallback([=](const std::string& n, const cv::Mat& mat){
            if (*_client_ready)
                _mat_merger->updateMat(n, mat);
            return *_client_ready;
        });
    }
    _old_mat_names = qnames;
}

void EasyGuiShowClient::dumpAndCompareNames(const std::vector<QString>& new_names){
    auto dump_change = [=](){
        COBOT_LOG.notice() << "cv Mat name changed!";
        for (const auto& name : new_names) {
            COBOT_LOG.message() << "MAT name: " << name.toLocal8Bit().constData();
        }
    };

    if (_old_mat_names.size() != new_names.size()) {
        dump_change();
    } else {
        for (size_t i = 0; i < new_names.size(); i++) {
            if (_old_mat_names[i] != new_names[i]) {
                dump_change();
                return;
            }
        }
    }
}

void EasyGuiShowClient::initShowClient(){
    if (_reload_timer == nullptr) {
        _reload_timer = new QTimer(this);
        _reload_timer->setInterval(100);
        connect(_reload_timer, &QTimer::timeout, this, &EasyGuiShowClient::reloadImageTargets);
        _reload_timer->start();
    }
}

void EasyGuiShowClient::processMatReaderStatus(const QString& readerMat, int status){
    if (status == (int) cobotsys::common::EasyCvMatReaderStatus::TargetImageDoesNotExist) {
    }
    if (status == (int) cobotsys::common::EasyCvMatReaderStatus::ImageWriterAlreadyReleased) {
        COBOT_LOG.notice() << "Image Publisher Exit. Retry";
        _mat_status[readerMat].isConnected = false;
    }
    if (status == (int) cobotsys::common::EasyCvMatReaderStatus::ReaderRequireRelease) {
        COBOT_LOG.notice() << "Reader free: " << readerMat.toLocal8Bit().constData();
        _mat_status[readerMat].isConnected = false;
    }
    if (status == (int) cobotsys::common::EasyCvMatReaderStatus::ReadSharedCvMatReady) {
        _mat_status[readerMat].isConnected = true;
    }
    if (status == (int) cobotsys::common::EasyCvMatReaderStatus::MatUpdated) {
        if (_mat_status[readerMat].isCvShowEnabled) {
            _mat_merger->imshow(readerMat.toLocal8Bit().constData());
        }
        emit clientDataUpdated();
    }
}

void EasyGuiShowClient::clearClientMat(){
    _mat_merger->clear();
}

QStringList EasyGuiShowClient::getConnectedMatNames() const{
    QStringList nameList;

    for (const auto& iter : _mat_status) {
        if (iter.second.isConnected) {
            nameList << iter.first;
        }
    }

    return nameList;
}

void EasyGuiShowClient::showWithOpenCvApi(const QString& s){
    auto iter = _mat_status.find(s);
    if (iter != _mat_status.end()) {
        bool old_status = iter->second.isCvShowEnabled;
        iter->second.isCvShowEnabled = !old_status;
        if (iter->second.isCvShowEnabled)
            _mat_merger->imshow(s.toLocal8Bit().constData());
        else
            cv::destroyWindow(s.toLocal8Bit().constData());
    }
}

bool EasyGuiShowClient::getCvMatViewStatus(const QString& s) const{
    auto iter = _mat_status.find(s);
    if (iter != _mat_status.end()) {
        return iter->second.isCvShowEnabled;
    }
    return false;
}

void EasyGuiShowClient::destoryAllCvShowWindow(){
    for (auto& iter : _mat_status) {
        iter.second.isCvShowEnabled = false;
        cv::destroyWindow(iter.first.toLocal8Bit().constData());
    }
}

MatMerger& EasyGuiShowClient::getInternalMatMerger(){
    return *_mat_merger.get();
}


EasyGuiShowClient::NamedImageStatus::NamedImageStatus(){
    isConnected = false;
    isCvShowEnabled = false;
}
