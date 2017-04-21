//
// Created by 潘绪洋 on 17-4-5.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_BASIC_LOGGER_WIDGET_H
#define COBOTSYS_BASIC_LOGGER_WIDGET_H


#include <QWidget>
#include <QPlainTextEdit>
#include <QVBoxLayout>
#include <QAction>
#include <QTimer>
#include <QStringList>
#include "cobotsys_logger.h"

class BasicLoggerWidget : public QWidget {
Q_OBJECT
public:
    BasicLoggerWidget(QWidget* parent = nullptr);
    ~BasicLoggerWidget();

protected:
    void updateTextToUI();
    void appendText(const std::string& message);
    void setupUi();

protected:
    QPlainTextEdit* m_plainTextEdit;
    QVBoxLayout* m_boxLayout;
    QTextCursor m_textCursor;
    QTimer* m_editUpdateTimer;
    QString m_cachedMessage;
};


#endif //COBOTSYS_BASIC_LOGGER_WIDGET_H
