//
// Created by 潘绪洋 on 17-4-18.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <qwt_plot_curve.h>
#include <qwt_plot.h>
#include <QApplication>

int main(int argc, char** argv) {
    QApplication a(argc, argv);

    QwtPlot* myPlot = new QwtPlot(QString("Two Curves"), nullptr);

// add curves
    QwtPlotCurve* curve1 = new QwtPlotCurve("Curve 1");
    QwtPlotCurve* curve2 = new QwtPlotCurve("Curve 2");

// connect or copy the data to the curves
//    curve1->setData(...);
//    curve2->setData(...);

    curve1->attach(myPlot);
    curve2->attach(myPlot);

// finally, refresh the plot
    myPlot->replot();
    myPlot->show();
    myPlot->resize(1000, 500);

    return a.exec();
}