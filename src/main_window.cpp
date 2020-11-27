/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <iostream>

#include "../include/localization_ui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindowDesign),
    qnode(argc, argv),
    alignStateStr_("false"),
    connectThread_(false),
    gpsBtnState_(false),
    setTimerState_(false),
    writeFlag_(false),
    timeCnt_(0),
    timeInterval_(0),
    odom_x_(0.0),
    odom_y_(0.0),
    odom_z_(0.0),
    gps_x_(0.0),
    gps_y_(0.0),
    gps_z_(0.0),
    gps_error_(0.0)
{
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));
    setWindowTitle("Localization control UI");
    qnode.init();

    connect(this, SIGNAL(pushInitialPose(bool)), &qnode, SLOT(sendInitialPose(bool)));
    connect(&qnode, SIGNAL(pushGPSData(double, double, double, double)), this, SLOT(updateGPSData(double, double, double, double)));
    connect(&qnode, SIGNAL(pushAlignState(bool)), this, SLOT(updateAlignState(bool)));
    connect(&qnode, SIGNAL(pushOdomData(double, double, double)), this, SLOT(updateOdomData(double, double, double)));
    connect(&qnode, SIGNAL(rosShutDown()), this, SLOT(close()));

    m_timer = new QTimer(this);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(onTimer()));

    connectThread_ = true;
    file_thread = new std::thread(&MainWindow::writeFile, this);

    ui->le_alignstate->setStyleSheet("color:red;");
}

MainWindow::~MainWindow()
{
    connectThread_ = false;
    if(file_thread->joinable())
        file_thread->join();
    delete file_thread;

    delete m_timer;
    delete ui;
}

void MainWindow::onTimer()
{
    if(timeCnt_ >= timeInterval_) {
        writeFlag_ = true;
        timeCnt_ = 0;
    }

    timeCnt_++;
    //std::cout << "time: " << timeCnt_ << std::endl;
}

void MainWindow::writeFile()
{
    char time_buf[100];
    time_t tm_time;

    string saveDirectory = std::getenv("HOME");
    saveDirectory += "/local_result.txt";
    std::cout << saveDirectory << endl;
    fout.open(saveDirectory);

    while(connectThread_)
    {
        if(writeFlag_)
        {
            time(&tm_time);
            strftime(time_buf, 100, "%Y%m%d_%H%M%S", localtime(&tm_time));
            fout << time_buf << endl;
            fout << "Align State: " << alignStateStr_ << endl;
            fout << "GPS: ";
            fout << gps_x_ << ", " << gps_y_ << ", " << gps_z_ << endl;
            fout << "Odom: ";
            fout << odom_x_ << ", " << odom_y_ << ", " << odom_z_ << endl;
            double accurate = sqrt(pow(gps_x_-odom_x_,2)+pow(gps_y_-odom_y_,2));
            fout << "GPS error: " << gps_error_*100.0 << "cm" << endl;
            fout << "Accurate: " << accurate*100.0 << "cm" << endl << endl;
            std::cout << "write!" << endl;
            writeFlag_ = false;
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }

    fout.close();
}

void MainWindow::updateGPSData(double x, double y, double z, double error)
{
    QString str;

    gps_x_ = x;
    gps_y_ = y;
    gps_z_ = z;
    gps_error_ = error;

    str = str.sprintf("%.3lf m", gps_x_);
    ui->le_gpsx->setText(str);
    str = str.sprintf("%.3lf m", gps_y_);
    ui->le_gpsy->setText(str);
    str = str.sprintf("%.3lf m", gps_z_);
    ui->le_gpsz->setText(str);
}

void MainWindow::updateAlignState(bool state)
{
    if(state) {
        alignStateStr_ = "true";
        ui->le_alignstate->setStyleSheet("color:green;");
    }
    else {
        alignStateStr_ = "false";
        ui->le_alignstate->setStyleSheet("color:red;");
    }
    ui->le_alignstate->setText(alignStateStr_.c_str());
}

void MainWindow::updateOdomData(double x, double y, double z)
{
    QString str;

    odom_x_ = x;
    odom_y_ = y;
    odom_z_ = z;

    str = str.sprintf("%.3lf m", odom_x_);
    ui->le_curx->setText(str);
    str = str.sprintf("%.3lf m", odom_y_);
    ui->le_cury->setText(str);
    str = str.sprintf("%.3lf m", odom_z_);
    ui->le_curz->setText(str);
}

void MainWindow::on_pb_initialpose_clicked()
{
    emit pushInitialPose(!gpsBtnState_);
}

void MainWindow::on_cb_settimer_clicked(bool checked)
{
    setTimerState_ = checked;
    ui->sb_time->setEnabled(setTimerState_);
    ui->pb_getgps->setCheckable(setTimerState_);
}

void MainWindow::on_pb_getgps_clicked(bool checked)
{
    if(setTimerState_)
    {
        timeInterval_ = ui->sb_time->text().toInt();
        if(checked) {
            m_timer->start(1000);
        }
        else {
            m_timer->stop();
        }
    }
    else {
        writeFlag_ = true;
    }
}

void MainWindow::on_actionGPS_off_toggled(bool arg1)
{
    gpsBtnState_ = arg1;
    ui->groupBox_2->setEnabled(!gpsBtnState_);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_S:
        ui->pb_initialpose->click();
        break;

    case Qt::Key_G:
        ui->actionGPS_off->setChecked(!gpsBtnState_);
        break;
    }
}
