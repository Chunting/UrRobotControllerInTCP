//
// Created by 潘绪洋 on 17-1-19.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_LOGGERWIDGET_H
#define PROJECT_LOGGERWIDGET_H

#include <QWidget>
#include <QTimer>
#include <QPlainTextEdit>
#include <cobotsys_gui_logger_highlighter.h>
#include "cobotsys_logger.h"

class LoggerViewWidget : public QWidget {
Q_OBJECT
public:
    LoggerViewWidget(QWidget *parent = nullptr);

    void clear();

    void bindCurrentProcessLogger();

    void setOpacity();
protected:
    void appendText(const std::string &entry, const std::string &message);
    void updateTextToUI();

protected:
    QPlainTextEdit *_plainTextEdit;
    QTextCursor _textCursor;
    QString _cachedMessage;
    QTimer *_editUpdateTimer;
    cobotsys::gui::LoggerHighlighter *_highlighter;
};

#endif //PROJECT_LOGGERWIDGET_H
