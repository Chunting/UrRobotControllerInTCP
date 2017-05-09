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
#include <QMenu>
#include <QStringList>
#include "cobotsys_logger.h"
#include <cobotsys_abstract_widget.h>
#include <mutex>

using cobotsys::AbstractWidget;

class BasicLoggerWidget : public AbstractWidget {
Q_OBJECT
public:
    BasicLoggerWidget();
    ~BasicLoggerWidget();

    virtual bool setup(const QString& configFilePath);

protected:
    void updateTextToUI();
    void appendText(const std::string& message);
    void setupUi();

    void customMenu();

    void parserLogTypeName(const QString& line_, QString& type_, QString& name_);

    void showLogType(const QString& type_, bool show_);
    void showLogName(const QString& name_, bool show_);

    void loopAllTextBlock(std::function<void(QTextBlock&)> func);

    bool isTypeVisible(const QString& type_);
    bool isNameVisible(const QString& name_);

    void addTextFilter(QMenu* menu);
    void addHideItems(QMenu* menu);

    void appendFilterMessage();

    void addConfigMenu(QMenu* menu);
    void saveConfig();
    void loadConfig();
protected:
    QPlainTextEdit* m_plainTextEdit;
    QVBoxLayout* m_boxLayout;
    QTextCursor m_textCursor;
    QTimer* m_editUpdateTimer;
    QString m_cachedMessage;
    std::mutex m_mutex;

    bool m_autoScrollBottom;
    bool m_enableFilter;
    bool m_enableGUIFilterMenu;

    std::map<QString, bool> m_typeFilter;
    std::map<QString, bool> m_nameFilter;

    QString m_configPath;
    QFont m_ctxMenuFont;
};


#endif //COBOTSYS_BASIC_LOGGER_WIDGET_H
