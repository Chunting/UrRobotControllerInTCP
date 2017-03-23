//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "simple_debug_gl_object.h"

using namespace cobotsys;

simple_debug_gl_object::simple_debug_gl_object(){
    m_transform.Identity();
    m_object.GenerateCube();
    m_object.AddOffset({0, 0, 1.0});
}

simple_debug_gl_object::~simple_debug_gl_object(){
}

void simple_debug_gl_object::Render(){
    glPushMatrix();
    glMultMatrixd(m_transform.Array());
    m_object.Render();
    glPopMatrix();
    AbstractRenderer::Render();
}
