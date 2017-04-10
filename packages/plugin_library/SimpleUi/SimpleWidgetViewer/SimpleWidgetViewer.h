//
// Created by 潘绪洋 on 17-4-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_SIMPLEWIDGETVIEWER_H
#define COBOTSYS_SIMPLEWIDGETVIEWER_H

#include "cobotsys.h"
#include "cobotsys_abstract_widget.h"
#include "ui_SimpleWidgetViewer.h"
#include "WidgetCloser.h"
#include <QStringList>
#include <QTimer>
#include <QTextCursor>

using namespace cobotsys;

class SimpleWidgetViewer : public AbstractWidget {
Q_OBJECT
public:
    SimpleWidgetViewer();
    virtual ~SimpleWidgetViewer();

    virtual bool setup(const QString& configFilePath);

public:
    void actionCreateWidget();
    void actionClear();
    void actionCreateWidgetNoJson();

protected:
    void refreshWidgetList();
    void resetCurObj();

    void createTextLogUi();
    void updateTextToUI();
    void appendText(const std::string& entry, const std::string& message);

protected:
    Ui::SimpleWidgetViewer ui;
    shared_ptr<AbstractObject> m_pWidget;
    WidgetCloser* m_closer;


    QTimer* m_editUpdateTimer;
    QString m_cachedMessage;
    QTextCursor m_textCursor;
};


#endif //COBOTSYS_SIMPLEWIDGETVIEWER_H
