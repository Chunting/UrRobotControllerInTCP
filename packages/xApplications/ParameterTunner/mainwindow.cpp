#include <QTimer>
#include <QLayout>
#include "mainwindow.h"
#include <qwt_slider.h>
MainWindow::MainWindow() : m_ticks(0.0) {

    d_calibButton = new QPushButton("Gravity Calib",this);
    d_connButton = new QPushButton("Device Disconnected", this);
    d_dragButton = new QPushButton("Drag stop",this);
    d_calibButton->setEnabled(false);
    d_dragButton->setEnabled(false);
    d_plot = new QwtPlot(QwtText("Drag App Monitor"), this);
    m_timer = new QTimer();
    QStringList signalList;//Add the signal that you want to monitor.
    signalList.push_back("m_force.force.x");
    signalList.push_back("m_force.force.y");
    signalList.push_back("m_force.force.z");
    signalList.push_back("m_force.torque.x");
    signalList.push_back("m_force.torque.y");
    signalList.push_back("m_force.torque.z");
    signalList.push_back("Force ee 0");
    signalList.push_back("Force ee 1");
    signalList.push_back("Force ee 2");
    signalList.push_back("Force ee 3");
    signalList.push_back("Force ee 4");
    signalList.push_back("Force ee 5");
    signalList.push_back("Force error 0");
    signalList.push_back("Force error 1");
    signalList.push_back("Force error 2");
    signalList.push_back("Force error 3");
    signalList.push_back("Force error 4");
    signalList.push_back("Force error 5");


    QStringList parameterList;//Add the signal that you want to monitor.
    parameterList.push_back("PID.P1-PID.P6");
    parameterList.push_back("PID.I1-PID.I6");
    parameterList.push_back("PID.D1-PID.D6");
    parameterList.push_back("PID.N1-PID.N6");
    parameterList.push_back("dead_zone_start");
    parameterList.push_back("dead_zone_end");

    //TODO 了解new方法何时被delete的。
    QHBoxLayout *mainLayout = new QHBoxLayout(this);
    QVBoxLayout *v1Layout = new QVBoxLayout();
    v1Layout->addWidget(d_plot, 10);
    QHBoxLayout *v1_h1Layout = new QHBoxLayout();

    v1Layout->addLayout(v1_h1Layout);
    QHBoxLayout *v2_h1Layout = new QHBoxLayout();

    for(int i=0;i<6;i++){
        if(i<3){
            d_cboSignal.push_back(new QComboBox(this));
            d_cboSignal[i]->clear();
            d_cboSignal[i]->addItems(signalList);

            d_lblSig.push_back(new QLabel(this));
            d_lblSig[i]->setText("0.0");

            v1_h1Layout->addWidget(d_cboSignal[i]);
            v1_h1Layout->addWidget(d_lblSig[i]);

        }
        d_sldBox.push_back(new SliderBox(4));
        v2_h1Layout->addWidget(d_sldBox[i]);
        m_param.push_back(PID_P+i);
        d_sldBox[i]->setNum(*m_param[i]);
       d_sldBox[i]->d_slider->setValue(*m_param[i]);
    }
    if (GlobalObjectFactory::instance()) {
        auto obj = GlobalObjectFactory::instance()->createObject("SimpleUiFactory, Ver 1.0", "BasicLoggerWidget");
        m_loggerWidget = std::dynamic_pointer_cast<QWidget>(obj);
        m_loggerWidget->setWindowFlags(Qt::Window);
    }
    v1Layout->addWidget(m_loggerWidget.get());

    d_cboSignal[0]->setCurrentIndex(0);
    d_cboSignal[1]->setCurrentIndex(1);
    d_cboSignal[2]->setCurrentIndex(2);
    m_sig.push_back(&m_force.force.x);
    m_sig.push_back(&m_force.force.y);
    m_sig.push_back(&m_force.force.z);


    QVBoxLayout *v2Layout = new QVBoxLayout();
    v2Layout->addLayout(v2_h1Layout);

    d_cboParam=new QComboBox(this);
    d_cboParam->clear();
    d_cboParam->addItems(parameterList);

    v2Layout->addWidget(d_cboParam);
    QHBoxLayout* v2_h2Layout =new QHBoxLayout();
    v2_h2Layout->addWidget(d_connButton);
    v2_h2Layout->addWidget(d_calibButton);
    v2_h2Layout->addWidget(d_dragButton);
    v2Layout->addLayout(v2_h2Layout);

    mainLayout->addLayout(v1Layout);
    mainLayout->addLayout(v2Layout);
    m_dragController.reset(new DragController());

    qRegisterMetaType<StdVector>("StdVector");
    qRegisterMetaType<MyWrench>("MyWrench");

    connect(d_connButton, &QPushButton::released, this, &MainWindow::onConnectDevice);
    connect(d_calibButton, &QPushButton::released, this, &MainWindow::onSettingCalib);
    connect(d_dragButton, &QPushButton::released, this, &MainWindow::onDragStart);

    connect(m_dragController.get(), SIGNAL(jointUpdated(StdVector)), this, SLOT(onJointUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(poseUpdated(StdVector)), this, SLOT(onPoseUpdated(StdVector)));
    connect(m_dragController.get(), SIGNAL(forceUpdated(MyWrench)), this, SLOT(onForceUpdated(MyWrench)));
    connect(d_cboParam,SIGNAL(currentIndexChanged(int)),this,SLOT(onParamGroupChanged(int)));

    connect(d_cboSignal[0],SIGNAL(currentIndexChanged(int)),this,SLOT(onSignal1Changed(int)));
    connect(d_cboSignal[1],SIGNAL(currentIndexChanged(int)),this,SLOT(onSignal2Changed(int)));
    connect(d_cboSignal[2],SIGNAL(currentIndexChanged(int)),this,SLOT(onSignal3Changed(int)));

    connect(d_sldBox[0]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide1ValueChanged(double)));
    connect(d_sldBox[1]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide2ValueChanged(double)));
    connect(d_sldBox[2]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide3ValueChanged(double)));
    connect(d_sldBox[3]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide4ValueChanged(double)));
    connect(d_sldBox[4]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide5ValueChanged(double)));
    connect(d_sldBox[5]->d_slider,SIGNAL(valueChanged(double)),this,SLOT(onSlide6ValueChanged(double)));

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
void MainWindow::signalChanged(int index,int SigId){
    switch(index){
        case 0:
            m_sig[SigId]=&m_force.force.x;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to m_force.force.x";
            break;
        case 1:
            m_sig[SigId]=&m_force.force.y;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to fm_force.force.y";
            break;
        case 2:
            m_sig[SigId]=&m_force.force.z;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to m_force.force.z";
            break;
        case 3:
            m_sig[SigId]=&m_force.torque.x;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to m_force.torque.x";
            break;
        case 4:
            m_sig[SigId]=&m_force.torque.y;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to m_force.torque.y";
            break;
        case 5:
            m_sig[SigId]=&m_force.torque.z;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to m_force.torque.z";
            break;
        case 6://force error 0
            m_sig[SigId]=m_dragController->force_ee;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 0";
            break;
        case 7://force error 1
            m_sig[SigId]=m_dragController->force_ee+1;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 1";
            break;
        case 8://force error 2
            m_sig[SigId]=m_dragController->force_ee+2;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 2";
            break;
        case 9://force error 3
            m_sig[SigId]=m_dragController->force_ee+3;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 3";
            break;
        case 10://force error 4
            m_sig[SigId]=m_dragController->force_ee+4;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 4";
            break;
        case 11://force error 5
            m_sig[SigId]=m_dragController->force_ee+5;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force ee 5";
            break;
        case 12://force error 0
            m_sig[SigId]=force_error;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 0";
            break;
        case 13://force error 1
            m_sig[SigId]=force_error+1;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 1";
            break;
        case 14://force error 2
            m_sig[SigId]=force_error+2;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 2";
            break;
        case 15://force error 3
            m_sig[SigId]=force_error+3;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 3";
            break;
        case 16://force error 4
            m_sig[SigId]=force_error+4;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 4";
            break;
        case 17://force error 5
            m_sig[SigId]=force_error+5;
            COBOT_LOG.notice()<<"Set Signal "<<SigId<<" to force error 5";
            break;
        default:
            COBOT_LOG.notice()<<"Signal "<<SigId<<" selected ee.";
            break;
    }
}
void MainWindow::onParamGroupChanged(int index){
//    parameterList.push_back("PID.P1-PID.P6");
//    parameterList.push_back("PID.I1-PID.I6");
//    parameterList.push_back("PID.D1-PID.D6");
//    parameterList.push_back("PID.N1-PID.N6");
//    parameterList.push_back("dead_zone_start");
//    parameterList.push_back("dead_zone_end");
    switch(index){
        case 0://PID.P1-PID.P6

            for(int i=0;i<6;i++)
            {
                m_param[i]=PID_P+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.P1-PID.P6 selected.";
            break;
        case 1://PID.I1-PID.I6

            for(int i=0;i<6;i++)
            {
                m_param[i]=PID_I+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.I1-PID.I6 selected.";
            break;
        case 2://PID.D1-PID.D6

            for(int i=0;i<6;i++)
            {
                m_param[i]=PID_D+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.D1-PID.P6 selected.";
            break;
        case 3://PID.N1-PID.N6

            for(int i=0;i<6;i++)
            {
                m_param[i]=PID_N+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.N1-PID.N6 selected.";
            break;
        case 4://dead_zone_start

            for(int i=0;i<6;i++)
            {
                m_param[i]=dead_zone_start+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.N1-PID.N6 selected.";
            break;
        case 5://dead_zone_end

            for(int i=0;i<6;i++)
            {
                m_param[i]=dead_zone_end+i;
                d_sldBox[i]->setNum(*m_param[i]);
                d_sldBox[i]->d_slider->setValue(*m_param[i]);
            }
            COBOT_LOG.notice()<<"PID.N1-PID.N6 selected.";
            break;
        default:
            COBOT_LOG.notice()<<"Parameter selected error.";
            break;
    }
}

void MainWindow::onSlide1ValueChanged(double value){
//    parameterList.push_back("PID.P1-PID.P6");
//    parameterList.push_back("PID.I1-PID.I6");
//    parameterList.push_back("PID.D1-PID.D6");
//    parameterList.push_back("PID.N1-PID.N6");
    *m_param[0]=value;
    //COBOT_LOG.notice()<<"Slide1 changed to "<<value;
}
void MainWindow::onSlide2ValueChanged(double value){
    *m_param[1]=value;
    //COBOT_LOG.notice()<<"Slide2 changed to "<<value;
}
void MainWindow::onSlide3ValueChanged(double value){
    *m_param[2]=value;
    //COBOT_LOG.notice()<<"Slide3 changed to "<<value;
}
void MainWindow::onSlide4ValueChanged(double value){
    *m_param[3]=value;
    //COBOT_LOG.notice()<<"Slide4 changed to "<<value;
}
void MainWindow::onSlide5ValueChanged(double value){
    *m_param[4]=value;
    //COBOT_LOG.notice()<<"Slide5 changed to "<<value;
}
void MainWindow::onSlide6ValueChanged(double value){
    *m_param[5]=value;
    //COBOT_LOG.notice()<<"Slide6 changed to "<<value;
}

void MainWindow::onSignal1Changed(int index){
    signalChanged(index,0);
}

void MainWindow::onSignal2Changed(int index){
    signalChanged(index,1);
}

void MainWindow::onSignal3Changed(int index){
    signalChanged(index,2);
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
    m_curve_y1.push_back(*m_sig[0]);
    m_curve_y2.push_back(*m_sig[1]);
    m_curve_y3.push_back(*m_sig[2]);

    curves[0]->setSamples(m_ticks.data(), m_curve_y1.data(), m_ticks.size());
    curves[1]->setSamples(m_ticks.data(), m_curve_y2.data(), m_ticks.size());
    curves[2]->setSamples(m_ticks.data(), m_curve_y3.data(), m_ticks.size());


    if (m_ticks.size() > 2) {
        d_plot->setAxisScale(QwtPlot::Axis::xBottom, m_ticks.front(), m_ticks.back());
    }

    ticks += 0.01;
    if((int)(ticks*100)%50==0){
        QString text;
        for(int i=0;i<3;i++){
            text.setNum( *m_sig[i], 'f', 2 );
            d_lblSig[i]->setText( text );
        }
    }
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

void MainWindow::onConnectDevice() {
    static bool runStatus = false;
    runStatus = !runStatus;
    if (runStatus) {

        m_timer->start(10);
        m_dragController->onStartDrag();
        d_connButton->setText("Device Connected");
        d_calibButton->setEnabled(true);
    } else {
        m_timer->stop();
        m_dragController->onStopDrag();//
        d_connButton->setText("Device Disonnected");
        //d_calibButton->setEnabled(false);
        //d_dragButton->setEnabled(false);
    }
}
void MainWindow::onSettingCalib(){
    m_dragController->setControllerStatus(DragController::CALIB);
    d_dragButton->setEnabled(true);//TODO 此处需要在标定完成后，再设置。

}
void MainWindow::onDragStart(){
    static bool dragStatus=false;
    dragStatus=!dragStatus;
    if(dragStatus){
        m_dragController->setControllerStatus(DragController::DRAG);
        d_dragButton->setText("Drag start");
    }else{
        m_dragController->setControllerStatus(DragController::IDLE);
        d_dragButton->setText("Drag stop");
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
//    static int count = 0;
//    count++;
//    if (count % 1000 == 0) {
//        COBOT_LOG.notice() << "Wrench:(" <<
//                           m_force.force.x << ", " <<
//                           m_force.force.y << ", " <<
//                           m_force.force.z << ", " <<
//                           m_force.torque.x << ", " <<
//                           m_force.torque.y << ", " <<
//                           m_force.torque.z << ")";
//    }
}
