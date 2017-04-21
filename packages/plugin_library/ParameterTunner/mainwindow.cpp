#include <qstatusbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qevent.h>
#include <qdatetime.h>
#include <qwt_plot_canvas.h>
#include "mainwindow.h"

MainWindow::MainWindow()
{
    d_dragButton= new QPushButton("Drag Stoped", this);


    d_plot = new QwtPlot(QwtText("Two Curves"), this);

    QHBoxLayout *hLayout = new QHBoxLayout( this );

    hLayout->addWidget( d_plot, 10 );
    hLayout->addWidget( d_dragButton);


    d_frameCount = new QLabel( this );




    m_dragController.reset(new DragController());

    qRegisterMetaType<StdVector>("StdVector");
    qRegisterMetaType<MyWrench >("MyWrench");
    connect(d_dragButton, &QPushButton::released, this, &MainWindow::dragAction);
   // connect(ui.btnStart, &QPushButton::released, m_dragController.get(), &DragController::onStartDrag);
//    connect(ui.btnStop, &QPushButton::released, m_dragController.get(), &DragController::onStopDrag);


    connect(m_dragController.get(), SIGNAL(jointUpdated(StdVector)), this, SLOT(onJointUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(poseUpdated(StdVector)), this, SLOT(onPoseUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(forceUpdated(MyWrench)), this, SLOT(onForceUpdated(MyWrench)));

    m_dragController->setup("CONFIG/UrRobotConfig/ur3_181_config.json");
}
void MainWindow::dragAction(){
    static bool runStatus=false;
    runStatus=!runStatus;
    if(runStatus){
        m_dragController->onStartDrag();
        d_dragButton->setText("Drag running");
    }else{
        m_dragController->onStopDrag();
        d_dragButton->setText("Drag stoped");
    }
}

void MainWindow::onJointUpdated(const StdVector &joints) {

}
void MainWindow::onPoseUpdated(const StdVector &xyzrpy){
    // Update to UI

}
void MainWindow::onForceUpdated(const Wrench &ptrWrench) {
    // Update to UI

}
