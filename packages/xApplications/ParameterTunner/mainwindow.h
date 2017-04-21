#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include "DragController.h"
#include <cobotsys_abstract_widget.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QTimer>
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
    void updatePlot();
    void initializePlot();
    QPushButton *d_dragButton;
    QwtPlot *d_plot;
    QVector<QwtPlotCurve*> curves;
    std::shared_ptr<DragController> m_dragController;
    QTimer* m_timer;
    std::vector<double> m_pose;
    std::vector<double> m_joints;
    Wrench m_force;
    std::vector<double> m_ticks;
    std::vector<double> m_curve_y1;
    std::vector<double> m_curve_y2;
    std::vector<double> m_curve_y3;
};

#endif
