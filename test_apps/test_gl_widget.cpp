//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <cobotsys_simple_world_renderer_widget.h>
#include <QApplication>
#include <simple_debug_gl_object.h>

int main(int argc, char** argv){
    QApplication a(argc, argv);

    cobotsys::SimpleWorldRendererWidget w;
    auto pobj = std::make_shared<simple_debug_gl_object>();

    w.resize(900, 500);
    w.AddRenderer(pobj);

    w.show();

    return a.exec();
}