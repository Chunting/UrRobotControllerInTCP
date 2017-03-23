//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#ifndef PROJECT_COBOTSYS_SIMPLE_WORLD_RENDERER_WIDGET_H
#define PROJECT_COBOTSYS_SIMPLE_WORLD_RENDERER_WIDGET_H


#include <QOpenGLWidget>
#include <GLTools/GL3DObject.h>
#include <StdTools/Timer.h>
#include <GLTools/GLCamera.h>
#include <cobotsys_abstract_renderer.h>
#include <QMutex>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTimer>

#define MAX_NUM_LIGHTS 8

namespace cobotsys {
class SimpleWorldRendererWidget : public QOpenGLWidget {
Q_OBJECT
protected:
    int mGLWidth;
    int mGLHeight;


    Chrono mChrono;

    PerformanceEstimator mRenderingTime;

    Chrono mFPSChrono;
    int mFPS;
    int mFPSCounter;

    QTimer* myPaintQtimer;

    QMutex mRenderingMutex;

    vector<AbstractRendererPtr> mRenderers;


    GLCamera mCamera;
    Vector3 mCameraInitPos;
    Vector3 mCameraInitLookAt;


    Qt::MouseButton mCurrentMouseButton;
    int mCurrentMouseX;
    int mCurrentMouseY;
    GLCamera::CameraMode mCameraMode;


    char mCurrentKey;

    float mBackgroundColor[4];

    bool bUseFog;
    float mFogColor[4];
    float mFogNear, mFogFar;

    bool bUseFloor;
    float mFloorStepSize;
    float mFloorStepCount;
    float mFloorColor[2][4];
    int mFloorCallListId;
    double mFloorHeight;
    int mNumRobots;

    bool bUseGrid;
    int mGridStepCount;
    int mGridHeightStepCount;
    float mGridColor[4];
    int mGridCallListId;


    int mAntialiasingLevel;

    int mNumLights;
    float mLightColor[MAX_NUM_LIGHTS][3][4];
    bool bUseLightPosition[MAX_NUM_LIGHTS];
    float mLightPosition[MAX_NUM_LIGHTS][4];
    bool bUseLightDirection[MAX_NUM_LIGHTS];
    float mLightDirection[MAX_NUM_LIGHTS][4];

    int mColorMaterial;

    GL3DObject mLightObject;
    GL3DObject mCameraObject;
    GL3DObject mCameraObjectLine;


    bool bInitGL;

    bool bShowHelp;
    bool bShowStats;
    bool bRenderShadows;
    bool bRenderOutlines;
    bool bRenderBBoxes;
    bool bRenderTransparency;
    bool bRenderCameraLookAt;
    bool bRenderFrames;

    bool bIsFullScreen;

    bool bCtrlModifierOn;
    bool bShiftModifierOn;

    bool bSlowFrameRate;

    bool bIsRecording;
    int mFrameNum;

    std::vector<int> mGLWidthList;
    std::vector<int> mGLHeightList;
    char* tmp_filename;

public:
    SimpleWorldRendererWidget(QWidget* parent = Q_NULLPTR, Qt::WindowFlags f = Qt::WindowFlags());
    virtual ~SimpleWorldRendererWidget();

    void AddRenderer(AbstractRendererPtr renderer);
    void ClearRenderer();
protected:
    void DrawFloor();
    void DrawGrid();
    void DrawCameraLookAt();
    void DrawStats();
    void DrawCredits();

    void loadDefaultConfig();
protected:
    virtual void initializeGL();
    virtual void paintGL();
    virtual void resizeGL(int w, int h);

    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
};
}

#endif //PROJECT_COBOTSYS_SIMPLE_WORLD_RENDERER_WIDGET_H
