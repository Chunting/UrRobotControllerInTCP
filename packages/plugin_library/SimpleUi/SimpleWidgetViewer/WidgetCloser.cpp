//
// Created by 潘绪洋 on 17-4-7.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "WidgetCloser.h"
#include <cobotsys_abstract_object.h>

bool WidgetCloser::eventFilter(QObject* obj, QEvent* event) {
    if (event->type() == QEvent::Close) {
        auto aobj = dynamic_cast<cobotsys::AbstractObject*>(obj);
        if (aobj) {
            Q_EMIT widgetClosed();
        }
    }
    return QObject::eventFilter(obj, event);
}
