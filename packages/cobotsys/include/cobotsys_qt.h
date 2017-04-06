//
// Created by 潘绪洋 on 17-2-21.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_QT_H
#define PROJECT_COBOTSYS_QT_H

#include <ostream>
#include <QString>
#include <QJsonObject>
#include <QJsonDocument>
#include <QImage>
#include <cobotsys_logger.h>

std::ostream& operator<<(std::ostream& oss, const QString& str);
std::ostream& operator<<(std::ostream& oss, const QJsonObject& obj);

#endif //PROJECT_COBOTSYS_QT_H
