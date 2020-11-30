#ifndef localization_ui_MAIN_WINDOW_H
#define localization_ui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include <QTimer>
#include <thread>
#include <fstream>
#include <chrono>

#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

using namespace std;

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();
    void writeFile();

Q_SIGNALS:
    void pushInitialPose(bool gps);

public slots:
    void updateGPSData(double x, double y, double z, double error);
    void updateAlignState(bool state);
    void updateOdomData(double x, double y, double z);
    void onTimer();

private slots:
    void on_pb_initialpose_clicked();
    void on_cb_settimer_clicked(bool checked);
    void on_pb_getgps_clicked(bool checked);
    void on_actionGPS_off_toggled(bool arg1);

private:
    Ui::MainWindowDesign *ui;
    QNode                qnode;
    QTimer               *m_timer;
    std::thread          *file_thread;
    ofstream             fout;

    string alignStateStr_;
    bool connectThread_;
    bool gpsBtnState_;
    bool setTimerState_;
    bool writeFlag_;
    int timeCnt_;
    int timeInterval_;

    double odom_x_, odom_y_, odom_z_;
    double gps_x_, gps_y_, gps_z_, gps_error_;
    double accuracy_;

// QWidget interface
protected:
    void keyPressEvent(QKeyEvent *event);
};

#endif // localization_ui_MAIN_WINDOW_H
