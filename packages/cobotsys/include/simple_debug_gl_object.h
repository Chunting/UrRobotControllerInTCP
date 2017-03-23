//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_SIMPLE_DEBUG_GL_OBJECT_H
#define PROJECT_SIMPLE_DEBUG_GL_OBJECT_H


#include <cobotsys_abstract_renderer.h>
#include <GLTools/GL3DObject.h>

class simple_debug_gl_object : public cobotsys::AbstractRenderer {
public:

    GL3DObject m_object;

    MathLib::Matrix4 m_transform;
public:
    simple_debug_gl_object();
    virtual ~simple_debug_gl_object();


    virtual void Render();
};


#endif //PROJECT_SIMPLE_DEBUG_GL_OBJECT_H
