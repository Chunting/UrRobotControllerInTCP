//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_ABSTRACT_RENDERER_H
#define PROJECT_COBOTSYS_ABSTRACT_RENDERER_H

#include <StdTools/XmlTree.h>
#include "cobotsys_abstract_object.h"
#include "MathLib/Vector3.h"

namespace cobotsys {
class AbstractRenderer;
typedef std::shared_ptr<AbstractRenderer> AbstractRendererPtr;

class AbstractRenderer : public AbstractObject {
public:
    std::vector<AbstractRendererPtr> mSubRenderers;
public:
    AbstractRenderer();
    virtual ~AbstractRenderer();

public:
    virtual void Free();
    virtual void Render();
    virtual void RenderShadow(
            int pass, const MathLib::Vector3& light, const MathLib::Vector3& observer);
    virtual void RenderOutline(const MathLib::Vector3& observer);
    virtual void RenderBoundingBox();

    void AddRenderer(AbstractRendererPtr renderer);

    // Courtesy of M Duvanel
    virtual bool ComputeRayIntersection(
            MathLib::Vector3 vecRay1,
            MathLib::Vector3 vecRay2,
            double& dFactor,
            bool bIntersections,
            unsigned* pShapeIndex){ return false; };

    virtual void AddShapeFromRay(
            unsigned uShapeIndex,
            MathLib::Vector3 vecRay1,
            MathLib::Vector3 vecRay2,
            double dFactor,
            double dSphereScale){}

    virtual void RemoveShape(unsigned uShapeIndex){}
};


}


#endif //PROJECT_COBOTSYS_ABSTRACT_RENDERER_H
