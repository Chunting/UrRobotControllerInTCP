//
// Created by 潘绪洋 on 17-3-22.
// Copyright (c) 2017 Wuhan Collaborative Robot Technology Co.,Ltd. All rights reserved.
//

#include <QtOpenGL/QGLFormat>
#include "cobotsys_simple_world_renderer_widget.h"

namespace cobotsys {
SimpleWorldRendererWidget::SimpleWorldRendererWidget(QWidget* parent, Qt::WindowFlags f) : QOpenGLWidget(parent, f){
    bInitGL = true;
//    mConfigTree = NULL;
//    recorded_pixels.resize(1000);

    GLint m_viewport[4];
    glGetIntegerv(GL_VIEWPORT, m_viewport);
//    rec_one_frame = NULL;
    tmp_filename = new char[25];

    //mConfigTree->Print();
//    LoadConfig(config);

    setWindowTitle("SimpleWorldRendererWidget (CTRL+<h> for help)");

//    mIdleTimer = new QTimer(this);
//    connect(mIdleTimer, SIGNAL(timeout()), this, SLOT(OnIdle()));
//    mIdleTimer->start(0);
//
    myPaintQtimer = new QTimer(this);
    connect(myPaintQtimer, &QTimer::timeout, [=](){ update(); });
    myPaintQtimer->start(33);

    mCurrentKey = 0;

    mFloorCallListId = 0;
    mGridCallListId = 0;


    mLightObject.GenerateSphere(8, 8);
    Matrix3 id;
    id.Identity();
    id *= 0.1;
    mLightObject.Transform(id);

    mCameraObject.GenerateSphere(16, 12);
    id *= 0.5;
    mCameraObject.Transform(id);

    mCameraObjectLine.GenerateCube();
    //id.Identity();
    id(0, 0) *= 0.1;
    id(1, 1) *= 0.1;
    id(2, 2) *= 200;
    mCameraObjectLine.Transform(id);

    mRenderingTime.SetCount(30);

    mGLWidth = 0;
    mGLHeight = 0;

    mFPS = 0;
    mFPSCounter = 0;
    mFPSChrono.Start();

    /*
    mPPSChrono.Start();
    mPPS        = 0;
    mPPSCounter = 0;
     */

    /*
    mProcessingPeriod = 0.001;
    mRunTime          = 0.0;

    mRunChrono.Start();
    mRunChrono.Pause();

    bIsPausing      = false;
    mTimeToPause    = 0.0;
    bIsPaused       = true;
     */

    bShowHelp = false;
    bShowStats = true;
    bRenderShadows = false;
    bRenderOutlines = false;
    bRenderBBoxes = false;
    bRenderTransparency = false;
    bRenderCameraLookAt = true;
    bCtrlModifierOn = false;
    bShiftModifierOn = false;
    bRenderFrames = true;

    bIsFullScreen = false;

    bIsRecording = false;

    loadDefaultConfig();

//    mSimInterface = NULL;
}

SimpleWorldRendererWidget::~SimpleWorldRendererWidget(){
}

void SimpleWorldRendererWidget::initializeGL(){
    glEnable(GL_DEPTH_TEST);

    for (int i = 0; i < mNumLights; i++) {
        glLightfv(GL_LIGHT0 + i, GL_AMBIENT, mLightColor[i][0]);
        glLightfv(GL_LIGHT0 + i, GL_DIFFUSE, mLightColor[i][1]);
        glLightfv(GL_LIGHT0 + i, GL_SPECULAR, mLightColor[i][2]);
        glEnable(GL_LIGHT0 + i);
    }
    for (int i = mNumLights; i < MAX_NUM_LIGHTS; i++) {
        glDisable(GL_LIGHT0 + i);
    }
    if (mNumLights > 0)
        glEnable(GL_LIGHTING);


    switch (mColorMaterial) {
        case 0:
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
            break;
        case 1:
            glColorMaterial(GL_FRONT_AND_BACK, GL_DIFFUSE);
            break;
        case 2:
            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
            break;
        case 3:
            glColorMaterial(GL_FRONT_AND_BACK, GL_SPECULAR);
            break;
        case 4:
            glColorMaterial(GL_FRONT_AND_BACK, GL_EMISSION);
            break;
    }
    glEnable(GL_COLOR_MATERIAL);


    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);


    if (mAntialiasingLevel == 1) {
        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
        glEnable(GL_POLYGON_SMOOTH);
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
    }
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_TEXTURE_2D);

    glEnable(GL_NORMALIZE);
    glClearColor(mBackgroundColor[0], mBackgroundColor[1], mBackgroundColor[2], mBackgroundColor[3]);

    if (bUseFog) {
        glFogi(GL_FOG_MODE, GL_LINEAR);//GL_EXP; // GL_EXP2,
        glFogfv(GL_FOG_COLOR, mFogColor);
        glFogf(GL_FOG_DENSITY, 0.01);
        glHint(GL_FOG_HINT, GL_DONT_CARE);
        glFogf(GL_FOG_START, mFogNear);
        glFogf(GL_FOG_END, mFogFar);
        glEnable(GL_FOG);
    }


    float zero[] = {0.0, 0.0, 0.0, 1.0};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, zero);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);    // Really Nice Perspective Calculations



    mCurrentMouseButton = Qt::NoButton;
    mCurrentMouseX = 0;
    mCurrentMouseY = 0;
    mCamera.SetPosition(mCameraInitPos);
    mCamera.SetLookAt(mCameraInitLookAt);
    mCamera.Move(0, 0, 0);
    mCamera.Apply();

    bInitGL = false;
}

void SimpleWorldRendererWidget::resizeGL(int w, int h){
    mGLWidth = w;
    mGLHeight = h;

    mCamera.SetViewport(w, h);
    mCamera.Apply();
}

void SimpleWorldRendererWidget::paintGL(){
    QMutexLocker locker(&mRenderingMutex);

    if (bInitGL) {
        initializeGL();
    }

    mChrono.Start();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glPushMatrix();
    glLoadIdentity();
    mCamera.Apply(false);

    for (int i = 0; i < mNumLights; i++) {
        if (bUseLightPosition[i]) {
            glLightfv(GL_LIGHT0 + i, GL_POSITION, mLightPosition[i]);
        }
        if (bUseLightDirection[i]) {
            glLightfv(GL_LIGHT0 + i, GL_SPOT_DIRECTION, mLightDirection[i]);
            glLightf(GL_LIGHT0 + i, GL_SPOT_CUTOFF, 30.0);
        }
    }
    glDisable(GL_LIGHTING);
    for (int i = 0; i < mNumLights; i++) {
        glColor3f(1, 1, 0);
        glPushMatrix();
        glTranslatef(mLightPosition[i][0], mLightPosition[i][1], mLightPosition[i][2]);
        mLightObject.Render();
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);


    if (bUseFloor)
        DrawFloor();

    if (bUseGrid)
        DrawGrid();

    for (unsigned int i = 0; i < mRenderers.size(); i++) {
        mRenderers[i]->Render();
    }

    if (bRenderCameraLookAt && (mCurrentMouseButton != Qt::NoButton))
        DrawCameraLookAt();

    if (bRenderOutlines) {
        for (unsigned int i = 0; i < mRenderers.size(); i++) {
            mRenderers[i]->RenderOutline(mCamera.m_position);
        }
    }
    if (bRenderBBoxes) {
        for (unsigned int i = 0; i < mRenderers.size(); i++) {
            mRenderers[i]->RenderBoundingBox();
        }
    }
    if (bRenderShadows) {
        for (int j = 0; j < mNumLights; j++) {
            Vector3 lp(mLightPosition[j][0], mLightPosition[j][1], mLightPosition[j][2]);

            Vector3 obs = mCamera.m_position;

            if (bUseLightPosition[j]) {
                GL3DObject::RenderShadowInit();
                for (unsigned int i = 0; i < mRenderers.size(); i++) {
                    for (int pass = 0; pass < 2; pass++) {
                        mRenderers[i]->RenderShadow(pass, lp, mCamera.m_position);
                    }
                }
                GL3DObject::RenderShadowEnd();
            }
        }
    }

//    if ((mSimInterface) && (mSimInterface->GetSystemState() == ModuleInterface::SYSSTATE_STARTED))
//        mSimInterface->Draw();

    glPopMatrix();


    if (bShowHelp) {
//        if ((mWorld == NULL) || (mWorld->IsEmpty())) {
//            DrawNoWorldMessage();
//        }
//        DrawHelp();
    } else {
        if (bShowStats) {
            DrawStats();
        }
//        if ((mWorld == NULL) || (mWorld->IsEmpty())) {
//            DrawNoWorldMessage();
//        }
    }


    mRenderingTime.AddMeasurement(REALTYPE(mChrono.ElapsedTimeUs()));

    mFPSCounter++;
    if (mFPSChrono.ElapsedTimeMs() > 1000) {
        mFPS = MAX(0, mFPSCounter - 1);
        mFPSCounter = 0;
        mFPSChrono.Start();
    }
}

void SimpleWorldRendererWidget::DrawCredits(){
    int fontSize = 14;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, mGLWidth / float(fontSize), mGLHeight / float(fontSize), 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

    glColor4f(0, 0, 0, 0.7);
    glBegin(GL_QUADS);
    glVertex2f(0, 0);
    glVertex2f(mGLWidth / float(fontSize), 0);
    glVertex2f(mGLWidth / float(fontSize), mGLHeight / float(fontSize));
    glVertex2f(0, mGLHeight / float(fontSize));
    glEnd();

    char txt[256];

    sprintf(txt, "RobotToolKit version 0.8 (c) 2011");
    GLT::DisplayText(0, 0, txt, 1);
    sprintf(txt, "  (or yet another robot toolkit)");
    GLT::DisplayText(0, 1, txt, 1);

    sprintf(txt, "Autor: Eric Sauser");
    GLT::DisplayText(0, 3, txt, 1);
    sprintf(txt, "  email: eric.sauser@a3.epfl.ch");
    GLT::DisplayText(0, 4, txt, 1);
    sprintf(txt, "Learning Algorithms and Systems Laboratory (LASA)");
    GLT::DisplayText(0, 6, txt, 1);
    sprintf(txt, "Swiss Federal Institute of Technology Lausanne (EPFL)");
    GLT::DisplayText(0, 7, txt, 1);


    sprintf(txt, "Credits:");
    GLT::DisplayText(0, 9, txt, 1);
    sprintf(txt, "  Physics simulation engine:");
    GLT::DisplayText(0, 10, txt, 1);
    sprintf(txt, "    Open Dynamic Engine (ODE v0.11.1) for simulation");
    GLT::DisplayText(0, 11, txt, 1);
    sprintf(txt, "      www.ode.org");
    GLT::DisplayText(0, 12, txt, 1);
    sprintf(txt, "    Bullet Physics Library (v2.76) for collision detection");
    GLT::DisplayText(0, 13, txt, 1);
    sprintf(txt, "      www.bulletphysics.org");
    GLT::DisplayText(0, 14, txt, 1);


    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_CULL_FACE);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void SimpleWorldRendererWidget::DrawFloor(){
    glEnable(GL_POLYGON_OFFSET_FILL);
    if (mCamera.m_position.cz() > 0) {
        glPolygonOffset(0.0f, 100.0f);
    } else {
        glPolygonOffset(0.0f, -100.0f);
    }

    if (mFloorCallListId > 0) {
        glCallList(mFloorCallListId);
    } else {
        if (mFloorCallListId == 0)
            mFloorCallListId = glGenLists(1);
        if (mFloorCallListId > 0)
            glNewList(mFloorCallListId, GL_COMPILE);

        float col[4];
        col[0] = 0.2;
        col[1] = 0.2;
        col[2] = 0.2;
        col[3] = 1.0;
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, col);
        glBegin(GL_QUADS);
        float start = -float(mFloorStepCount) * 0.5 * mFloorStepSize;
        float startx = start;
        for (int i = 0; i < mFloorStepCount; i++) {
            float starty = start;
            for (int j = 0; j < mFloorStepCount; j++) {
                int cId = (i + j) % 2;
                glColor4f(mFloorColor[cId][0], mFloorColor[cId][1], mFloorColor[cId][2], mFloorColor[cId][3]);
                glNormal3f(0, 0, 1);
                glVertex3f(startx, starty, mFloorHeight);
                glVertex3f(startx + mFloorStepSize, starty, mFloorHeight);
                glVertex3f(startx + mFloorStepSize, starty + mFloorStepSize, mFloorHeight);
                glVertex3f(startx, starty + mFloorStepSize, mFloorHeight);
                glNormal3f(0, 0, -1);
                glVertex3f(startx, starty + mFloorStepSize, mFloorHeight);
                glVertex3f(startx + mFloorStepSize, starty + mFloorStepSize, mFloorHeight);
                glVertex3f(startx + mFloorStepSize, starty, mFloorHeight);
                glVertex3f(startx, starty, mFloorHeight);
                starty += mFloorStepSize;
            }
            startx += mFloorStepSize;
        }
        glEnd();
        if (mFloorCallListId > 0)
            glEndList();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
}

void SimpleWorldRendererWidget::DrawGrid(){
    if (mGridCallListId > 0) {
        glCallList(mGridCallListId);
    } else {
        if (mGridCallListId == 0)
            mGridCallListId = glGenLists(1);
        if (mGridCallListId > 0)
            glNewList(mGridCallListId, GL_COMPILE);

        glLineStipple(2, 0xAAAA);
        glEnable(GL_LINE_STIPPLE);
        glBegin(GL_LINES);
        glColor4f(mGridColor[0], mGridColor[1], mGridColor[2], mGridColor[3]);
        for (int i = -mGridStepCount; i <= mGridStepCount; i += 2) {
            for (int j = -mGridStepCount; j <= mGridStepCount; j += 2) {
                glVertex3f(mFloorStepSize * i, mFloorStepSize * j, 0);
                glVertex3f(mFloorStepSize * i, mFloorStepSize * j, mFloorStepSize * mGridHeightStepCount);
            }
        }
        for (int i = -mGridStepCount; i <= mGridStepCount; i += 2) {
            for (int j = 0; j <= mGridHeightStepCount; j += 2) {
                glVertex3f(mFloorStepSize * i, mFloorStepSize * -mGridStepCount, mFloorStepSize * j);
                glVertex3f(mFloorStepSize * i, mFloorStepSize * mGridStepCount, mFloorStepSize * j);
                glVertex3f(mFloorStepSize * -mGridStepCount, mFloorStepSize * i, mFloorStepSize * j);
                glVertex3f(mFloorStepSize * mGridStepCount, mFloorStepSize * i, mFloorStepSize * j);
            }
        }
        glEnd();
        glDisable(GL_LINE_STIPPLE);

        if (mFloorCallListId > 0)
            glEndList();
    }
}

void SimpleWorldRendererWidget::DrawCameraLookAt(){
    glColor4f(0, 0.8, 1, 0.6);
    glPushMatrix();
    glTranslatef(mCamera.m_lookAtPoint[0], mCamera.m_lookAtPoint[1], mCamera.m_lookAtPoint[2]);
    mCameraObject.Render();
    mCameraObjectLine.Render();
    glRotatef(90, 1, 0, 0);
    mCameraObjectLine.Render();
    glRotatef(90, 0, 1, 0);
    mCameraObjectLine.Render();
    glPopMatrix();
}

void SimpleWorldRendererWidget::DrawStats(){
    int fontSize = 13;

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, mGLWidth / float(fontSize), mGLHeight / float(fontSize), 0);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_CULL_FACE);

    char txt[256];

//    if (mSimInterface) {
//        sprintf(txt, "RunTime:  %1.1f (%1.1f)", mSimInterface->GetSimulationTime(),
//                mSimInterface->GetSimulationRunTime());
//        GLT::DisplayText(0, 0, txt, 1);
//    }
    sprintf(txt, "FPS: %d", mFPS);
    GLT::DisplayText(0, 1, txt, 1);

    sprintf(txt, "LookAt: %6.2f %6.2f %6.2f",
            mCamera.m_lookAtPoint.x(),
            mCamera.m_lookAtPoint.y(),
            mCamera.m_lookAtPoint.z());
    GLT::DisplayText(0, 2, txt, 1);
//    if (mSimInterface) {
//        sprintf(txt, "PPS: %d", mSimInterface->GetPPS());
//        GLT::DisplayText(0, 2, txt, 1);
//    }
    sprintf(txt, "Timers:");
    GLT::DisplayText(0, 4, txt, 1);

    sprintf(txt, " Rendering:  %5.1fus (%1.2fs)", mRenderingTime.GetTime(), mRenderingTime.GetTime() * mFPS * 1e-6);
    GLT::DisplayText(0, 5, txt, 1);

//    if (mSimInterface) {
//        sprintf(txt, " Processing: %1.0fus (%1.2fs)", mSimInterface->GetPTime(),
//                mSimInterface->GetPTime() * mSimInterface->GetPPS() * 1e-6);
//        GLT::DisplayText(0, 6, txt, 1);
//    }
//    if (mWorld != NULL) {
//        sprintf(txt, "Modules:");
//        GLT::DisplayText(0, 8, txt, 1);
//        int cpos = 9;
//
//        const vector<WorldInterface *> &winterfaces = mWorld->GetWorldInterfaces();
//        for (int i = 0; i < int(winterfaces.size()); i++) {
//            sprintf(txt, " %s: %1.0f - %1.0f us",
//                    winterfaces[i]->GetInterfaceName().c_str(),
//                    winterfaces[i]->GetPerformanceEstimatorUpdateCore().GetTime() * 1e6,
//                    winterfaces[i]->GetPerformanceEstimatorUpdate().GetTime() * 1e6);
//            GLT::DisplayText(0, cpos++, txt, 1);
//        }
//        const vector<Robot *> &wrobots = mWorld->GetRobots();
//        for (int j = 0; j < int(wrobots.size()); j++) {
//            const vector<RobotInterface *> &rinterfaces = wrobots[j]->GetRobotInterfaces();
//            for (int i = 0; i < int(rinterfaces.size()); i++) {
//                sprintf(txt, " %s: %1.0f - %1.0f us",
//                        rinterfaces[i]->GetInterfaceName().c_str(),
//                        rinterfaces[i]->GetPerformanceEstimatorUpdateCore().GetTime() * 1e6,
//                        rinterfaces[i]->GetPerformanceEstimatorUpdate().GetTime() * 1e6);
//                GLT::DisplayText(0, cpos++, txt, 1);
//            }
//        }
//    }
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_CULL_FACE);
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
}

void SimpleWorldRendererWidget::keyPressEvent(QKeyEvent* event){
    if (event->modifiers() & Qt::ShiftModifier) {
        bShiftModifierOn = true;
    } else {
        bShiftModifierOn = false;
    }

    if (event->modifiers() & Qt::ControlModifier) {
        bCtrlModifierOn = true;

        switch (event->key()) {
            case Qt::Key_H:
                bShowHelp = !bShowHelp;
                break;
            case Qt::Key_F:
                bSlowFrameRate = !bSlowFrameRate;
                if (bSlowFrameRate) {
                    myPaintQtimer->setInterval(200);
                } else {
                    myPaintQtimer->setInterval(33);
                }
                break;
            case Qt::Key_R:
                mCamera.SetPosition(mCameraInitPos);
                mCamera.SetLookAt(mCameraInitLookAt);
                mCamera.Move(0, 0, 0);
                mCamera.Apply();
                break;
            case Qt::Key_1:
                bShowStats = !bShowStats;
                break;
            case Qt::Key_2:
                bRenderShadows = !bRenderShadows;
                break;
            case Qt::Key_3:
                bRenderOutlines = !bRenderOutlines;
                break;
            case Qt::Key_4:
                bRenderBBoxes = !bRenderBBoxes;
                break;
            case Qt::Key_6:
                bRenderCameraLookAt = !bRenderCameraLookAt;
                break;
            case Qt::Key_7:
                bRenderFrames = !bRenderFrames;
                break;
        }
    } else {
        bCtrlModifierOn = false;
        //QGLWidget::keyPressEvent(event);
    }
    //QGLWidget::keyPressEvent(event);
}

void SimpleWorldRendererWidget::keyReleaseEvent(QKeyEvent* event){
    if (event->modifiers() & Qt::ShiftModifier) {
        bShiftModifierOn = true;
    } else {
        bShiftModifierOn = false;
    }

    if (event->modifiers() & Qt::ControlModifier) {
        bCtrlModifierOn = true;
    } else {
        bCtrlModifierOn = false;
    }
}

void SimpleWorldRendererWidget::mouseMoveEvent(QMouseEvent* event){
    if (!bShiftModifierOn) {
        if (mCurrentMouseButton == Qt::LeftButton) {
            int halfHeight = height() / 2;
            int dx = event->x() - mCurrentMouseX;
            int dy = event->y() - mCurrentMouseY;
            if (event->y() > halfHeight && !bCtrlModifierOn) dx = -dx;
            mCamera.Move((float) dx, -(float) dy, 0);
            mCurrentMouseX = event->x();
            mCurrentMouseY = event->y();
        }
        if (mCurrentMouseButton == Qt::RightButton) {
            mCamera.Move(0, 0, (float) (event->y() - mCurrentMouseY));
            mCurrentMouseX = event->x();
            mCurrentMouseY = event->y();
        }
    }
}

#define GLBW_BTNUP      0
#define GLBW_BTNDOWN    1
#define GLBW_LEFTBTN    0
#define GLBW_RIGHTBTN   1

void SimpleWorldRendererWidget::mousePressEvent(QMouseEvent* event){
    if (mCurrentMouseButton == Qt::NoButton) {
        if (!bShiftModifierOn) {
            if (bCtrlModifierOn) {
                mCameraMode = GLCamera::CAMMODE_FreeMove;
            } else {
                mCameraMode = GLCamera::CAMMODE_Centered;
            }
            mCamera.SetCameraMode(mCameraMode);

            mCurrentMouseButton = event->button();
            mCurrentMouseX = event->x();
            mCurrentMouseY = event->y();
        }
    }

    // Duduche's mods in this if...
    if (bShiftModifierOn && (!bCtrlModifierOn)) {
        bool bIntersects = (event->button() == Qt::RightButton);

        // We get the ray corresponding to the user's click
        Vector3 vecRay1, vecRay2;
        mCamera.GetRayVectors(event->x(), event->y(), vecRay1, vecRay2);

        //std::cout << "Clicked coordinates are " << mCurrentMouseX << ", " << mCurrentMouseY;
        //std::cout << ", button clicked is " << ( bIntersects ? "Right" : "Left" ) << std::endl;
        //std::cout << "Ray vectors are " << vecRay1.x() << ", " << vecRay1.y() << ", " << vecRay1.z();
        //std::cout << " and " << vecRay2.x() << ", " << vecRay2.y() << ", " << vecRay2.z() << std::endl;

        // Now let's find which object the user clicked
        double dFactor, dSmallestFactor = R_INFINITY;
        unsigned uClickedObjectIndex;
        unsigned uClickedShapeIndexTmp, uClickedShapeIndex;
        for (unsigned int i = 0; i < mRenderers.size(); i++) {
            if (mRenderers[i]->ComputeRayIntersection(vecRay1, vecRay2, dFactor, bIntersects, &uClickedShapeIndexTmp)) {
                if (dFactor < dSmallestFactor) {
                    dSmallestFactor = dFactor;
                    uClickedObjectIndex = i;
                    uClickedShapeIndex = uClickedShapeIndexTmp;
                    //std::cout << "Validating intersection, renderer index = " << uClickedObjectIndex << std::endl;
                }
            }
        }

        if (dSmallestFactor != R_INFINITY) {
            //std::cout << "Intersection found, index " << uClickedObjectIndex << std::endl;

            if (!bIntersects) {
                double dSphereScale = 0.005;
                mRenderers[uClickedObjectIndex]->AddShapeFromRay(uClickedShapeIndex, vecRay1, vecRay2, dSmallestFactor,
                                                                 dSphereScale);
            } else {
                mRenderers[uClickedObjectIndex]->RemoveShape(uClickedShapeIndex);
            }
        }
    }
}

void SimpleWorldRendererWidget::mouseReleaseEvent(QMouseEvent* event){
    if (mCurrentMouseButton == event->button()) {
        mCurrentMouseButton = Qt::NoButton;
    }
}

#define SET_COLOR4(array, r, g, b, a)   {(array)[0] = (r); (array)[1] = (g); (array)[2] = (b); (array)[3] = (a);}
#define COPY_COLOR4(array, src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = (src)[3];}
#define COPY_COLOR3(array, src)   {(array)[0] = (src)[0]; (array)[1] = (src)[1]; (array)[2] = (src)[2]; (array)[3] = 1.0;}

void SimpleWorldRendererWidget::loadDefaultConfig(){
    SET_COLOR4(mBackgroundColor, 0.0, 0.0, 0.1, 1.0);

    bUseFog = true;
    SET_COLOR4(mFogColor, 0.0, 0.0, 0.1, 1.0);
    mFogNear = 8.0;
    mFogFar = 12.0;

    bUseFloor = true;
    mFloorStepSize = 0.5;
    mFloorStepCount = 40;
    mFloorHeight = 0;
    SET_COLOR4(mFloorColor[0], 0.4, 0.4, 0.4, 1.0);
    SET_COLOR4(mFloorColor[1], 0.1, 0.1, 0.1, 1.0);

    mAntialiasingLevel = 0;

    mNumLights = 2;

    SET_COLOR4(mLightColor[0][0], 0.5, 0.5, 0.5, 1.0);
    SET_COLOR4(mLightColor[0][1], 1.0, 1.0, 1.0, 1.0);
    SET_COLOR4(mLightColor[0][2], 0.0, 0.0, 0.0, 1.0);
    SET_COLOR4(mLightPosition[0], 8.0, 8.0, 10.0, 1.0);
    SET_COLOR4(mLightDirection[0], -8.0, -8.0, -10.0, 1.0);
    bUseLightDirection[0] = true;
    bUseLightPosition[0] = true;

    SET_COLOR4(mLightColor[1][0], 0.0, 0.0, 0.0, 1.0);
    SET_COLOR4(mLightColor[1][1], 0.8, 0.8, 0.8, 1.0);
    SET_COLOR4(mLightColor[1][2], 0.0, 0.0, 0.0, 1.0);
    SET_COLOR4(mLightPosition[1], -8.0, -8.0, 10.0, 1.0);
    SET_COLOR4(mLightDirection[1], 8.0, 8.0, -10.0, 1.0);
    bUseLightDirection[1] = true;
    bUseLightPosition[1] = true;

    bUseGrid = false;
    mGridStepCount = 1 * 2;
    mGridHeightStepCount = 2 * 2;
    SET_COLOR4(mGridColor, 1.0, 1.0, 0.0, 1.0);


    mColorMaterial = 2;

    mCameraInitLookAt.Set(0, 0, 0.5);
    mCameraInitPos.Set(3, -3, 1);

    bInitGL = true;
}

void SimpleWorldRendererWidget::AddRenderer(AbstractRendererPtr renderer){
    if (renderer)
        mRenderers.push_back(renderer);
}

void SimpleWorldRendererWidget::ClearRenderer(){
    mRenderers.clear();
}
}