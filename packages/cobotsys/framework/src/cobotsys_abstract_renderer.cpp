//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include "cobotsys_abstract_renderer.h"

namespace cobotsys {
AbstractRenderer::AbstractRenderer(){
}

AbstractRenderer::~AbstractRenderer(){
    Free();
}

void AbstractRenderer::Free(){
    mSubRenderers.clear();
}

void AbstractRenderer::Render(){
    for (int i = 0; i < int(mSubRenderers.size()); i++) {
        mSubRenderers[i]->Render();
    }
}

void AbstractRenderer::RenderShadow(int pass, const MathLib::Vector3& light, const MathLib::Vector3& observer){
    for (int i = 0; i < int(mSubRenderers.size()); i++) {
        mSubRenderers[i]->RenderShadow(pass, light, observer);
    }
}

void AbstractRenderer::RenderOutline(const MathLib::Vector3& observer){
    for (int i = 0; i < int(mSubRenderers.size()); i++) {
        mSubRenderers[i]->RenderOutline(observer);
    }
}

void AbstractRenderer::RenderBoundingBox(){
    for (int i = 0; i < int(mSubRenderers.size()); i++) {
        mSubRenderers[i]->RenderBoundingBox();
    }
}

void AbstractRenderer::AddRenderer(AbstractRendererPtr renderer){
    mSubRenderers.push_back(renderer);
}

bool AbstractRenderer::setup(const QString& configFilePath){
    return true;
}
}