//
// Created by 潘绪洋 on 17-4-25.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef COBOTSYS_MATVIEWER_H
#define COBOTSYS_MATVIEWER_H

#include <QWidget>
#include <MatMerger.h>
#include <QPaintEvent>
#include <QPainter>
#include <QTimer>

class MatViewer : public QWidget {
Q_OBJECT
public:
    MatViewer(QWidget* parent = nullptr);
    virtual ~MatViewer();

    MatMerger& getMatMerger() { return m_matMerger; }

protected:
    virtual void paintEvent(QPaintEvent* event);

protected:
    MatMerger m_matMerger;
    QTimer* m_updateTimer;
};


#endif //COBOTSYS_MATVIEWER_H
