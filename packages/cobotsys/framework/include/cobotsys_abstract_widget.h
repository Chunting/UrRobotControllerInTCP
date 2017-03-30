//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_WIDGET_H
#define PROJECT_COBOTSYS_ABSTRACT_WIDGET_H


#include <cobotsys_abstract_object.h>
#include <QWidget>

namespace cobotsys {
/**
 * 创建这个类只是为了把在其他动态库里的Widget动态的创建出来。这个类本身没有什么。
 */
class AbstractWidget : public QWidget, public AbstractObject {
public:
    AbstractWidget();
    virtual ~AbstractWidget();
};
}

#endif //PROJECT_COBOTSYS_ABSTRACT_WIDGET_H
