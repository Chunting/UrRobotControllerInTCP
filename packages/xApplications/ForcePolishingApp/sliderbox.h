//
// Created by 杨帆 on 17-5-11.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef _SLIDER_BOX_H_
#define _SLIDER_BOX_H_ 1

#include <qwidget.h>

class QLabel;
class QwtSlider;

class SliderBox: public QWidget
{
    Q_OBJECT
public:
    SliderBox( int sliderType, QWidget *parent = NULL );

    QwtSlider *d_slider;
public Q_SLOTS:
    void setNum( double v );

private:
    QwtSlider *createSlider( int sliderType) const;


    QLabel *d_label;
};

#endif
