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
    gpsState_(false),
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
    setWindowTitle("Localization Control UI");
    qnode.init();

    setWindowFlags(Qt::WindowStaysOnTopHint);

    connect(this, SIGNAL(pushInitialPose(bool)), &qnode, SLOT(sendInitialPose(bool)));
    connect(this, SIGNAL(pushLocalizationOff(bool)), &qnode, SLOT(sendLocalizationOff(bool)));
    connect(this, SIGNAL(pushVisibleFlag(bool)), &qnode, SLOT(sendVisibleFlag(bool)));
    connect(this, SIGNAL(pushLineInfo(double, double, int, int, bool)), &qnode, SLOT(sendMakeFlag(double, double, int, int, bool)));
    connect(&qnode, SIGNAL(pushGPSData(double, double, double, double)), this, SLOT(updateGPSData(double, double, double, double)));
    connect(&qnode, SIGNAL(pushAlignState(bool)), this, SLOT(updateAlignState(bool)));
    connect(&qnode, SIGNAL(pushOdomData(double, double, double, double)), this, SLOT(updateOdomData(double, double, double, double)));
    connect(&qnode, SIGNAL(pushTotalPoints(int)), this, SLOT(updateTotalPoints(int)));
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

    bool openFlag = false;
    string saveDirectory = std::getenv("HOME");
    saveDirectory += "/local_result.txt";
    ROS_INFO("Logging file: %s", saveDirectory.c_str());    

    while(connectThread_)
    {
        if(writeFlag_)
        {
            if(!openFlag)
            {
                fout.open(saveDirectory);
                openFlag = true;
            }

            time(&tm_time);
            strftime(time_buf, 100, "%Y%m%d_%H%M%S", localtime(&tm_time));
            fout << time_buf << endl;
            fout << "Align State: " << alignStateStr_ << endl;
            fout << "GPS: ";
            fout << gps_x_ << ", " << gps_y_ << ", " << gps_z_ << endl;
            fout << "Odom: ";
            fout << odom_x_ << ", " << odom_y_ << ", " << odom_z_ << endl;
            //double accurate = sqrt(pow(gps_x_-odom_x_,2)+pow(gps_y_-odom_y_,2));
            fout << "GPS error: " << gps_error_*100.0 << "cm" << endl;
            fout << "Accurate: " << accuracy_*100.0 << "cm" << endl << endl;
            std::cout << "write!" << endl;
            writeFlag_ = false;
        }

        this_thread::sleep_for(chrono::milliseconds(1));
    }

    fout.close();
    openFlag = false;
}

void MainWindow::updateGPSData(double x, double y, double z, double error)
{
    gpsState_ = true;

    gps_x_ = x;
    gps_y_ = y;
    gps_z_ = z;
    gps_error_ = error;

    QString str;
    str = str.sprintf("%.3lf m", gps_x_);
    ui->le_gpsx->setText(str);
    str = str.sprintf("%.3lf m", gps_y_);
    ui->le_gpsy->setText(str);
    str = str.sprintf("%.3lf m", gps_z_);
    ui->le_gpsz->setText(str);

    if(gps_error_ < 1.0) {
      str = str.sprintf("%.3lf cm", gps_error_*100.0);
    }
    else {
      str = str.sprintf("%.3lf m", gps_error_);
    }
    ui->le_gpserror->setText(str);
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

void MainWindow::updateOdomData(double x, double y, double z, double time)
{
    QString str;

    odom_x_ = x;
    odom_y_ = y;
    odom_z_ = z;

    double distance = sqrt(pow(odom_x_-pre_odom_x_,2)+pow(odom_y_-pre_odom_y_,2));
    double speed = distance / (time-pre_odom_time_);

    str = str.sprintf("%.3lf m", odom_x_);
    ui->le_curx->setText(str);
    str = str.sprintf("%.3lf m", odom_y_);
    ui->le_cury->setText(str);
    str = str.sprintf("%.3lf m", odom_z_);
    ui->le_curz->setText(str);

    if(gpsState_)
      accuracy_ = sqrt(pow(gps_x_-odom_x_,2)+pow(gps_y_-odom_y_,2));

    if(accuracy_ < 1.0) {
      str = str.sprintf("%.3lf cm", accuracy_*100);
    }
    else {
      str = str.sprintf("%.3lf m", accuracy_);
    }
    ui->le_accuracy->setText(str);

    str = str.sprintf("%.1lf m/s", floor(speed*10)/10); // round off one decimal place
    ui->le_speed->setText(str);

    pre_odom_x_ = odom_x_;
    pre_odom_y_ = odom_y_;
    pre_odom_z_ = odom_z_;
    pre_odom_time_ = time;
}

void MainWindow::updateTotalPoints(int count)
{
    ui->le_tpoint->setText(QString::number(count));
}

void MainWindow::on_pb_initialpose_clicked()
{
    emit pushInitialPose(gpsState_);
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

void MainWindow::on_action_GPS_Off_toggled(bool arg1)
{
    ui->groupBox_2->setEnabled(!arg1);
    ui->le_accuracy->setEnabled(!arg1);
    ui->le_gpserror->setEnabled(!arg1);
}

void MainWindow::on_action_Localization_Off_toggled(bool arg1)
{
    emit pushLocalizationOff(arg1);
}

void MainWindow::on_cb_visible_clicked(bool checked)
{
    emit pushVisibleFlag(checked);
}

void MainWindow::on_pb_add_clicked()
{
    double distance = ui->le_distance->text().toDouble();
    double degree = ui->le_degree->text().toDouble();
    int start_point = ui->le_spoint->text().toInt();
    int end_point = ui->le_epoint->text().toInt();
    bool dir_flag;

    if(distance < 0)
    {
      distance = abs(distance);
      ui->le_distance->setText(QString::number(distance));
    }
    if(degree < 0)
    {
      degree = abs(degree);
      ui->le_degree->setText(QString::number(degree));
    }
    if(start_point < 0)
    {
      start_point = abs(start_point);
      ui->le_spoint->setText(QString::number(start_point));
    }
    if(end_point < 0)
    {
      end_point = abs(end_point);
      ui->le_epoint->setText(QString::number(end_point));
    }

    if("Forward" == ui->cb_direction->currentText()) dir_flag = true;
    else dir_flag = false;

    emit pushLineInfo(distance, degree, start_point, end_point, dir_flag);
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_S:
        ui->pb_initialpose->click();
        break;
    }
}
