/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <XmlRpcException.h>

#include "../include/localization_ui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv),
    gps_error_(0.0),
    gps_odomX_(0.0),
    gps_odomY_(0.0),
    gps_odomZ_(0.0),
    odomX_(0.0),
    odomY_(0.0),
    odomZ_(0.0)
{}

QNode::~QNode() {
    if(ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc, init_argv, "localization_ui");
    ROS_INFO("\033[1;36m----> UI node started.\033[0m");

    while ( ! ros::master::check() ) {
        ROS_WARN("Check ROS MASTER!");
        sleep(1.0);
    }
    ros::start();

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    initPubAndSub(nh);

    start();
    return true;
}

void QNode::run() {
    ros::Rate loop_rate(100);

    while ( ros::ok() )
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    Q_EMIT rosShutDown();
}


void QNode::initPubAndSub(ros::NodeHandle node_handle)
{
    novatel_bestutm_sub_ = node_handle.subscribe("pwk7/bestutm", 1, &QNode::bestUtmCallback, this);
    align_state_sub_     = node_handle.subscribe("align_state", 1, &QNode::alignStateCallback, this);
    odom_sub_            = node_handle.subscribe("odom", 1, &QNode::odomCallback, this);
    odom_gps_sub_        = node_handle.subscribe("gps/odom", 1, &QNode::odomGPSCallback, this);
    local_gps_client_    = node_handle.serviceClient<hdl_localization::initGPS>("local/gps");
}

void QNode::bestUtmCallback(const novatel_gps_msgs::NovatelUtmPosition::ConstPtr &msg)
{
    int zone = static_cast<int>(msg->lon_zone_number);
    double x = msg->easting;
    double y = msg->northing;
    double z = msg->height;

    double x_std = msg->easting_sigma;
    double y_std = msg->northing_sigma;

    gps_error_ = sqrt(pow(x_std,2)+pow(y_std,2));
}

void QNode::alignStateCallback(const std_msgs::Bool::ConstPtr &msg)
{
    emit pushAlignState(msg->data);
}

void QNode::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odomX_ = msg->pose.pose.position.x;
    odomY_ = msg->pose.pose.position.y;
    odomZ_ = msg->pose.pose.position.z;

    emit pushOdomData(odomX_, odomY_, odomZ_);
    emit pushGPSData(gps_odomX_, gps_odomY_, gps_odomZ_, gps_error_);
}

void QNode::odomGPSCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    gps_odomX_ = msg->pose.pose.position.x;
    gps_odomY_ = msg->pose.pose.position.y;
    gps_odomZ_ = msg->pose.pose.position.z;
}

void QNode::sendInitialPose(bool gps)
{
    if(gps)
    {
        hdl_localization::initGPS srv;
        srv.request.lat = gps_odomX_;
        srv.request.lon = gps_odomY_;
        srv.request.gps = true;

        if (local_gps_client_.call(srv))
        {
            ROS_INFO("Result: %d", srv.response.success);
        }
        else
        {
            ROS_ERROR("Failed to call service local gps");
        }
    }
    else
    {
        hdl_localization::initGPS srv;
        srv.request.lat = 0.0;
        srv.request.lon = 0.0;
        srv.request.gps = false;

        if (local_gps_client_.call(srv))
        {
            ROS_INFO("Result: %d", srv.response.success);
        }
        else
        {
            ROS_ERROR("Failed to call service local gps");
        }
    }
}
