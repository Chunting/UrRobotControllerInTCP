//
// Created by 潘绪洋 on 17-2-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_EASYGUISHOW_H
#define PROJECT_EASYGUISHOW_H

#include <QObject>
#include <QTimer>
#include <easy_cv_mat_reader.h>
#include "MatMerger.h"

// 这个对象负责把easy_gui_show的图片都获取出来，并且缓存到MatMerger里去。


class EasyGuiShowClient : public QObject {
Q_OBJECT
public:
    EasyGuiShowClient(QObject *parent = nullptr);
    ~EasyGuiShowClient();


    bool loadLayoutConfig(const cv::FileStorage &fs);
    void draw(QPainter &painter);
    void initShowClient();

    void clearClientMat();

    void showWithOpenCvApi(const QString &s);
    void destoryAllCvShowWindow();

    QStringList getConnectedMatNames() const;
    bool getCvMatViewStatus(const QString &s) const;
Q_SIGNALS:
    void matReaderStatus(const QString &readerMat, int status);
    void clientDataUpdated();
protected:
    void processMatReaderStatus(const QString &readerMat, int status);
    void reloadImageTargets();
    void dumpAndCompareNames(const std::vector<QString> &new_names);

protected:
    struct NamedImageStatus {
        bool isConnected;
        bool isCvShowEnabled;
        std::shared_ptr<cobotsys::common::EasyCvMatHolder> holder;
        NamedImageStatus();
    };
protected:
    std::shared_ptr<MatMerger> _mat_merger;
    cobotsys::common::EasyCvMatReader _easy_cv_mat_reader;
    std::vector<QString> _old_mat_names;
    std::map<QString, NamedImageStatus> _mat_status;
    QTimer *_reload_timer;
    bool _no_data_warning;
    std::shared_ptr<bool> _client_ready;
};


#endif //PROJECT_EASYGUISHOW_H
