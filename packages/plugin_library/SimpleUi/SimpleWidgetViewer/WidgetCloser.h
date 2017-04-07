//
// Created by 潘绪洋 on 17-4-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_WIDGETCLOSER_H
#define COBOTSYS_WIDGETCLOSER_H

#include <QObject>
#include <QEvent>


class WidgetCloser : public QObject {
Q_OBJECT
public:

Q_SIGNALS:
    void widgetClosed();
protected:
    bool eventFilter(QObject* obj, QEvent* event);
};


#endif //COBOTSYS_WIDGETCLOSER_H
