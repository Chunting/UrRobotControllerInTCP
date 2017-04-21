#ifndef _MAIN_WINDOW_H_
#define _MAIN_WINDOW_H_

#include <qmainwindow.h>
#include "DragController.h"
#include <cobotsys_abstract_widget.h>


class Plot;
class Panel;
class QLabel;
class Settings;

class MainWindow: public QMainWindow
{
    Q_OBJECT

public:
    MainWindow( QWidget *parent = NULL );
    virtual bool eventFilter( QObject *, QEvent * );

private Q_SLOTS:
    void applySettings( const Settings & );
public Q_SLOTS:
    void onJointUpdated(const StdVector &joints);
    void onPoseUpdated(const StdVector &xyzrpy);
    void onForceUpdated(const MyWrench &ptrWrench);
private:
    void dragAction();
    QPushButton *d_dragButton;
    Plot *d_plot;
    Panel *d_panel;
    QLabel *d_frameCount;
    std::shared_ptr<DragController> m_dragController;
};

#endif
