//
// Created by 潘绪洋 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "MatViewer.h"


MatViewer::MatViewer(QWidget* parent) : QWidget(parent) {
    m_updateTimer = new QTimer(this);
    m_updateTimer->setInterval(50);
    connect(m_updateTimer, &QTimer::timeout, [=]() { this->update(); });
    m_updateTimer->start();
}

MatViewer::~MatViewer() {
}

void MatViewer::paintEvent(QPaintEvent* event) {
    QPainter painter(this);

    QWidget::paintEvent(event);

    m_matMerger.draw(painter);
}
