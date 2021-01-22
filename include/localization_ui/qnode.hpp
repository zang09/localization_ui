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
#include <visualization_msgs/Marker.h>
#include <novatel_gps_msgs/NovatelUtmPosition.h>
#include <hdl_localization/initGPS.h>
#include <rviz_flag_plugin/PointArray.h>

#include "utm.h"

#define RAD2DEG  180/PI
#define DEG2RAD  PI/180

struct Line
{
  geometry_msgs::Point start_point;
  geometry_msgs::Point end_point;
  double angle_rad;
  double angle_deg;
};

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
    void trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg);

Q_SIGNALS:
    void rosShutDown();
    void pushGPSData(double, double, double, double);
    void pushAlignState(bool);
    void pushOdomData(double, double, double);
    void pushTotalPoints(int);

public Q_SLOTS:
    void sendInitialPose(bool);
    void sendVisibleFlag(bool);
    void sendMakeFlag(double, double, int, int, bool);

private:
    int init_argc;
    char** init_argv;

    ros::Subscriber novatel_bestutm_sub_;
    ros::Subscriber align_state_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber odom_gps_sub_;
    ros::Subscriber trajectory_sub_;
    ros::Publisher  make_flag_pub_;
    ros::Publisher  clear_flag_pub_;
    ros::Publisher  visible_flag_pub_;

    ros::ServiceClient local_gps_client_;

    std::vector<Line> line_info_;

    double gps_error_;
    double gps_odomX_, gps_odomY_, gps_odomZ_;
    double odomX_, odomY_, odomZ_;
};


#endif /* localization_ui_QNODE_HPP_ */
