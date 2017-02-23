//
// Created by 潘绪洋 on 17-2-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_qt.h"


std::ostream &operator<<(std::ostream &oss, const QString &str){
    oss << str.toLocal8Bit().constData();
    return oss;
}