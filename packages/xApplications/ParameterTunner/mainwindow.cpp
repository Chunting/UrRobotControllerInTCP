#include <QTimer>
#include <QLayout>
#include "mainwindow.h"
#include "sliderbox.h"

MainWindow::MainWindow() : m_ticks(0.0) {


    d_dragButton = new QPushButton("Drag Stoped", this);
    d_plot = new QwtPlot(QwtText("Drag App Monitor"), this);
    m_timer = new QTimer();

    //TODO 了解new方法何时被delete的。
    QHBoxLayout *mainLayout = new QHBoxLayout(this);

    QVBoxLayout *v1Layout = new QVBoxLayout(this);
    v1Layout->addWidget(d_plot, 10);

    QHBoxLayout *v1_h1Layout = new QHBoxLayout(this);

    d_cboSignal1=new QComboBox(this);
    d_cboSignal2=new QComboBox(this);
    d_cboSignal3=new QComboBox(this);

    d_lblSig1=new QLabel(this);
    d_lblSig2=new QLabel(this);
    d_lblSig3=new QLabel(this);
    QStringList signalList;//Add the signal that you want to monitor.
    signalList.push_back("Optoforce force x");
    signalList.push_back("Optoforce force y");
    signalList.push_back("Optoforce force z");
    signalList.push_back("Optoforce torque x");
    signalList.push_back("Optoforce torque y");
    signalList.push_back("Optoforce torque z");

    d_cboSignal1->clear();
    d_cboSignal1->addItems(signalList);
    d_lblSig1->setText("0.0");

    m_sig1=&m_force.force.x;
    m_sig2=&m_force.force.y;
    m_sig3=&m_force.force.z;

    d_cboSignal2->clear();
    d_cboSignal2->addItems(signalList);
    d_lblSig2->setText("0.0");

    d_cboSignal3->clear();
    d_cboSignal3->addItems(signalList);
    d_lblSig3->setText("0.0");

    v1_h1Layout->addWidget(d_cboSignal1);
    v1_h1Layout->addWidget(d_lblSig1);
    v1_h1Layout->addWidget(d_cboSignal2);
    v1_h1Layout->addWidget(d_lblSig2);
    v1_h1Layout->addWidget(d_cboSignal3);
    v1_h1Layout->addWidget(d_lblSig3);
    v1Layout->addLayout(v1_h1Layout);

    QHBoxLayout *h1Layout = new QHBoxLayout(this);
    h1Layout->addWidget( new SliderBox(4));
    h1Layout->addWidget( new SliderBox(4));
    h1Layout->addWidget( new SliderBox(4));
    h1Layout->addWidget( new SliderBox(4));
    h1Layout->addWidget( new SliderBox(4));
    h1Layout->addWidget( new SliderBox(4));

    QVBoxLayout *v2Layout = new QVBoxLayout(this);
    v2Layout->addLayout(h1Layout);

    QStringList parameterList;//Add the signal that you want to monitor.
    parameterList.push_back("PID.P1-PID.P6");
    parameterList.push_back("PID.I1-PID.I6");
    parameterList.push_back("PID.D1-PID.D6");
    parameterList.push_back("PID.N1-PID.N6");

    d_cboParam=new QComboBox(this);
    d_cboParam->clear();
    d_cboParam->addItems(parameterList);
    v2Layout->addWidget(d_cboParam);
    v2Layout->addWidget(d_dragButton);

    mainLayout->addLayout(v1Layout);
    mainLayout->addLayout(v2Layout);
    m_dragController.reset(new DragController());

    qRegisterMetaType<StdVector>("StdVector");
    qRegisterMetaType<MyWrench>("MyWrench");

    connect(d_dragButton, &QPushButton::released, this, &MainWindow::dragAction);
    connect(m_dragController.get(), SIGNAL(jointUpdated(StdVector)), this, SLOT(onJointUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(poseUpdated(StdVector)), this, SLOT(onPoseUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(forceUpdated(MyWrench)), this, SLOT(onForceUpdated(MyWrench)));
    //connect(d_cboParam.&QComboBox:)
    connect(m_timer, &QTimer::timeout, this, &MainWindow::updatePlot);
    m_dragController->setup("CONFIG/UrRobotConfig/ur3_180_config.json");
    initializePlot();
    m_joints.clear();
    m_pose.clear();
    for (int i = 0; i < 6; i++) {
        m_joints.push_back(0.0);
        m_pose.push_back(0.0);
    }
}

void MainWindow::updatePlot() {
    static double ticks = 0;
    if (m_ticks.size() >= 2000) {
        m_ticks.erase(m_ticks.begin());
        m_curve_y1.erase(m_curve_y1.begin());
        m_curve_y2.erase(m_curve_y2.begin());
        m_curve_y3.erase(m_curve_y3.begin());
    }
    m_ticks.push_back(ticks);
    m_curve_y1.push_back(*m_sig1);
    m_curve_y2.push_back(*m_sig2);
    m_curve_y3.push_back(*m_sig3);

    curves[0]->setSamples(m_ticks.data(), m_curve_y1.data(), m_ticks.size());
    curves[1]->setSamples(m_ticks.data(), m_curve_y2.data(), m_ticks.size());
    curves[2]->setSamples(m_ticks.data(), m_curve_y3.data(), m_ticks.size());


    if (m_ticks.size() > 2) {
        d_plot->setAxisScale(QwtPlot::Axis::xBottom, m_ticks.front(), m_ticks.back());
    }

    ticks += 0.01;
    d_plot->replot();
}

void MainWindow::initializePlot() {
//    std::vector<double> x;
//    std::vector<double> y;
//    for(int i=0;i<500;i++){
//        x.push_back((double)i*0.002);
//        y.push_back(0.0);
//    }
    curves.append(new QwtPlotCurve("curve1"));
    curves.append(new QwtPlotCurve("curve2"));
    curves.append(new QwtPlotCurve("curve3"));
    curves[0]->setSamples(m_ticks.data(), m_curve_y1.data(), m_ticks.size());
    curves[1]->setSamples(m_ticks.data(), m_curve_y2.data(), m_ticks.size());
    curves[2]->setSamples(m_ticks.data(), m_curve_y3.data(), m_ticks.size());
    curves[0]->setPen(QPen(Qt::red));
    curves[1]->setPen(QPen(Qt::green));
    curves[2]->setPen(QPen(Qt::yellow));
    curves[0]->attach(d_plot);
    curves[1]->attach(d_plot);
    curves[2]->attach(d_plot);
}

void MainWindow::dragAction() {
    static bool runStatus = false;
    runStatus = !runStatus;
    if (runStatus) {
        m_dragController->onStartDrag();
        m_timer->start(10);
        d_dragButton->setText("Drag running");
    } else {
        m_timer->stop();
        m_dragController->onStopDrag();
        d_dragButton->setText("Drag stoped");
    }
}

void MainWindow::onJointUpdated(const StdVector &joints) {
    m_joints = joints;
//    COBOT_LOG.notice()<<"Joint Updated0"<<m_joints[0]*180.0/M_PI;
//    COBOT_LOG.notice()<<"Joint Updated1"<<m_joints[1]*180.0/M_PI;
//    COBOT_LOG.notice()<<"Joint Updated2"<<m_joints[2]*180.0/M_PI;
//    COBOT_LOG.notice()<<"Joint Updated3"<<m_joints[3]*180.0/M_PI;
//    COBOT_LOG.notice()<<"Joint Updated4"<<m_joints[4]*180.0/M_PI;
//    COBOT_LOG.notice()<<"Joint Updated5"<<m_joints[5]*180.0/M_PI;
//    static int count=0;
//    count++;
//    if(count%100==0){
//        COBOT_LOG.notice()<<"count:"<<count;
//    }
}

void MainWindow::onPoseUpdated(const StdVector &xyzrpy) {
    m_pose = xyzrpy;
    //COBOT_LOG.notice()<<"Pose Updated";
    // Update to UI
//    static int count=0;
//    count++;
//    if(count%100==0){
//        COBOT_LOG.notice()<<"count:"<<count;
//    }
}

void MainWindow::onForceUpdated(const Wrench &ptrWrench) {
    // Update to UI
    m_force = ptrWrench;
    //COBOT_LOG.notice()<<"Force Updated";
    static int count = 0;
    count++;
    if (count % 1000 == 0) {
        COBOT_LOG.notice() << "Wrench:(" <<
                           m_force.force.x << ", " <<
                           m_force.force.y << ", " <<
                           m_force.force.z << ", " <<
                           m_force.torque.x << ", " <<
                           m_force.torque.y << ", " <<
                           m_force.torque.z << ")";
    }
}
