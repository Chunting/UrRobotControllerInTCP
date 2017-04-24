#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include "DragController.h"
#include "sliderbox.h"
#include <cobotsys_abstract_widget.h>
#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <QTimer>

#include <QLabel>
#include <QComboBox>

class Plot;
class QLabel;

class MainWindow: public QWidget
{
    Q_OBJECT
public:
    MainWindow( );

public Q_SLOTS:
    void onJointUpdated(const StdVector &joints);
    void onPoseUpdated(const StdVector &xyzrpy);
    void onForceUpdated(const MyWrench &ptrWrench);
    void onParamGroupChanged(int index);
    void onSignal1Changed(int index);
    void onSignal2Changed(int index);
    void onSignal3Changed(int index);

    void onSlide1ValueChanged(double value);
    void onSlide2ValueChanged(double value);
    void onSlide3ValueChanged(double value);
    void onSlide4ValueChanged(double value);
    void onSlide5ValueChanged(double value);
    void onSlide6ValueChanged(double value);


private:
    void onConnectDevice();
    void onSettingCalib();
    void onDragStart();
    void updatePlot();
    void initializePlot();



    QPushButton *d_connButton;
    QPushButton *d_calibButton;
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


    std::vector<QLabel*> d_lblSig;//3
    std::vector<double*> m_sig;//3
    std::vector<SliderBox*> d_sldBox;//6
    std::vector<double*> m_param;//6
    int m_paramGroupIndex;
    std::vector<QComboBox*> d_cboSignal;//3
    QComboBox* d_cboParam;
    std::shared_ptr<QWidget> m_loggerWidget;
};

#endif
