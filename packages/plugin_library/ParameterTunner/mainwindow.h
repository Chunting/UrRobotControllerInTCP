#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include <qmainwindow.h>
#include "DragController.h"
#include <cobotsys_abstract_widget.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>

class Plot;
class Panel;
class QLabel;
class Settings;

class MainWindow: public QWidget
{
    Q_OBJECT

public:
    MainWindow( );

public Q_SLOTS:
    void onJointUpdated(const StdVector &joints);
    void onPoseUpdated(const StdVector &xyzrpy);
    void onForceUpdated(const MyWrench &ptrWrench);
private:
    void dragAction();
    QPushButton *d_dragButton;
    QwtPlot *d_plot;
    QVector<QwtPlotCurve*> curves;
    QLabel *d_frameCount;
    std::shared_ptr<DragController> m_dragController;
};

#endif
