#ifndef localization_ui_QNODE_HPP_
#define localization_ui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <QThread>

#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <hdl_localization/initGPS.h>

#include "utm.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT

public:
    QNode(int argc, char** argv );
    virtual ~QNode();
    bool init();
    void run();    
    void initParams(ros::NodeHandle);
    void initPubAndSub(ros::NodeHandle);
    void bestUtmCallback(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr &msg);
    void alignStateCallback(const std_msgs::Bool::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void odomGPSCallback(const nav_msgs::Odometry::ConstPtr &msg);

Q_SIGNALS:
    void rosShutDown();
    void pushGPSData(double, double, double, double);
    void pushAlignState(bool);
    void pushOdomData(double, double, double);

public Q_SLOTS:
    void sendInitialPose(bool);

private:
    int init_argc;
    char** init_argv;

    ros::Subscriber novatel_bestutm_sub_;
    ros::Subscriber align_state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber odom_gps_sub_;

    ros::ServiceClient local_gps_client_;

    double mapX_, mapY_, mapZ_;
    double odomX_, odomY_, odomZ_;
};


#endif /* localization_ui_QNODE_HPP_ */
