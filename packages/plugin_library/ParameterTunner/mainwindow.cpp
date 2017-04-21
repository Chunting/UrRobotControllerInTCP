#include <QTimer>
#include <QLayout>
#include "mainwindow.h"

MainWindow::MainWindow():m_ticks(0.0)
{
    d_dragButton= new QPushButton("Drag Stoped", this);
    d_plot = new QwtPlot(QwtText("Two Curves"), this);

    m_timer = new QTimer();

    //TODO 了解new方法何时被delete的。
    QHBoxLayout *hLayout = new QHBoxLayout( this );

    hLayout->addWidget( d_plot, 10 );
    hLayout->addWidget( d_dragButton);

    m_dragController.reset(new DragController());

    qRegisterMetaType<StdVector>("StdVector");
    qRegisterMetaType<MyWrench >("MyWrench");

    connect(d_dragButton, &QPushButton::released, this, &MainWindow::dragAction);
    connect(m_dragController.get(), SIGNAL(jointUpdated(StdVector)), this, SLOT(onJointUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(poseUpdated(StdVector)), this, SLOT(onPoseUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(forceUpdated(MyWrench)), this, SLOT(onForceUpdated(MyWrench)));
    connect(m_timer, &QTimer::timeout,this,&MainWindow::updatePlot);
    m_dragController->setup("CONFIG/UrRobotConfig/ur3_181_config.json");
    initializePlot();
}
void MainWindow::updatePlot(){
    static double ticks=0;
    if(m_ticks.size()>=500){
        m_ticks.erase(m_ticks.begin());
        m_curve_y1.erase(m_curve_y1.begin());
        m_curve_y2.erase(m_curve_y2.begin());
        m_curve_y3.erase(m_curve_y3.begin());
    }
    m_ticks.push_back(ticks);
    m_curve_y1.push_back(m_force.force.x);
    m_curve_y2.push_back(m_force.force.y);
    m_curve_y3.push_back(m_force.force.z);
    curves[0]->setSamples(m_ticks.data(),m_curve_y1.data(),m_ticks.size());
    curves[1]->setSamples(m_ticks.data(),m_curve_y2.data(),m_ticks.size());
    curves[2]->setSamples(m_ticks.data(),m_curve_y3.data(),m_ticks.size());


    if (m_ticks.size() > 2) {
        d_plot->setAxisScale(QwtPlot::Axis::xBottom, m_ticks.front(), m_ticks.back());
    }

    ticks+=0.01;
    d_plot->replot();
}
void MainWindow::initializePlot(){
//    std::vector<double> x;
//    std::vector<double> y;
//    for(int i=0;i<500;i++){
//        x.push_back((double)i*0.002);
//        y.push_back(0.0);
//    }
    curves.append(new QwtPlotCurve("curve1"));
    curves.append(new QwtPlotCurve("curve2"));
    curves.append(new QwtPlotCurve("curve3"));
    curves[0]->setSamples(m_ticks.data(),m_curve_y1.data(),m_ticks.size());
    curves[1]->setSamples(m_ticks.data(),m_curve_y2.data(),m_ticks.size());
    curves[2]->setSamples(m_ticks.data(),m_curve_y3.data(),m_ticks.size());
    curves[0]->setPen(QPen(Qt::red));
    curves[1]->setPen(QPen(Qt::green));
    curves[2]->setPen(QPen(Qt::gray));
    curves[0]->attach(d_plot);
    curves[1]->attach(d_plot);
    curves[2]->attach(d_plot);
}
void MainWindow::dragAction(){
    static bool runStatus=false;
    runStatus=!runStatus;
    if(runStatus){
        m_dragController->onStartDrag();
        m_timer->start(10);
        d_dragButton->setText("Drag running");
    }else{
        m_timer->stop();
        m_dragController->onStopDrag();
        d_dragButton->setText("Drag stoped");
    }
}

void MainWindow::onJointUpdated(const StdVector &joints) {
    m_joints=joints;
}
void MainWindow::onPoseUpdated(const StdVector &xyzrpy){
    m_pose=xyzrpy;
    // Update to UI
}
void MainWindow::onForceUpdated(const Wrench &ptrWrench) {
    // Update to UI
    m_force=ptrWrench;
}
