//
// Created by 潘绪洋 on 17-3-23.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <extra2.h>
#include "cobotsys_abstract_widget.h"


namespace cobotsys {
AbstractWidget::AbstractWidget() : QWidget(nullptr) {
}

AbstractWidget::~AbstractWidget() {
    INFO_DESTRUCTOR(this);
}
}