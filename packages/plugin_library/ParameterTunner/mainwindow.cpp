#include <qstatusbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qevent.h>
#include <qdatetime.h>
#include <qwt_plot_canvas.h>
#include "mainwindow.h"

MainWindow::MainWindow( QWidget *parent ):
    QMainWindow( parent )
{
    QWidget *w = new QWidget( this );
    d_dragButton= new QPushButton("Drag Stoped", w);


    d_plot = new QwtPlot(QwtText("Two Curves"), w);

    QHBoxLayout *hLayout = new QHBoxLayout( w );

    hLayout->addWidget( d_plot, 10 );
    hLayout->addWidget( d_dragButton);
    setCentralWidget( w );

    d_frameCount = new QLabel( this );
    statusBar()->addWidget( d_frameCount, 10 );




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
bool MainWindow::eventFilter( QObject *object, QEvent *event )
{
    if ( object == d_plot->canvas() && event->type() == QEvent::Paint )
    {
        static int counter;
        static QTime timeStamp;

        if ( !timeStamp.isValid() )
        {
            timeStamp.start();
            counter = 0;
        }
        else
        {
            counter++;

            const double elapsed = timeStamp.elapsed() / 1000.0;
            if ( elapsed >= 1 )
            {
                QString fps;
                fps.setNum( qRound( counter / elapsed ) );
                fps += " Fps";

                d_frameCount->setText( fps );

                counter = 0;
                timeStamp.start();
            }
        }
    }

    return QMainWindow::eventFilter( object, event );
}

void MainWindow::applySettings( const Settings &settings )
{

    // the canvas might have been recreated
    d_plot->canvas()->removeEventFilter( this );
    d_plot->canvas()->installEventFilter( this );
}

void MainWindow::onJointUpdated(const StdVector &joints) {

}
void MainWindow::onPoseUpdated(const StdVector &xyzrpy){
    // Update to UI

}
void MainWindow::onForceUpdated(const Wrench &ptrWrench) {
    // Update to UI

}
