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
    align_state_sub_     = node_handle.subscribe("hdl_localization/align_state", 1, &QNode::alignStateCallback, this);
    odom_sub_            = node_handle.subscribe("hdl_localization/odom", 1, &QNode::odomCallback, this);
    odom_gps_sub_        = node_handle.subscribe("hdl_localization/gps_odom", 1, &QNode::odomGPSCallback, this);
    trajectory_sub_      = node_handle.subscribe("hdl_localization/globalmap/trajectory", 1, &QNode::trajectoryCallback, this);
    make_flag_pub_       = node_handle.advertise<rviz_flag_plugin::PointArray>("rviz/make_flag", 100, true);
    clear_flag_pub_      = node_handle.advertise<std_msgs::Bool>("rviz/clear_flag", 1);
    visible_flag_pub_    = node_handle.advertise<std_msgs::Bool>("rviz/visible_flag", 1);

    local_gps_client_    = node_handle.serviceClient<hdl_localization::initGPS>("hdl_localization/init_gps");
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
}

void QNode::odomGPSCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    gps_odomX_ = msg->pose.pose.position.x;
    gps_odomY_ = msg->pose.pose.position.y;
    gps_odomZ_ = msg->pose.pose.position.z;

    emit pushGPSData(gps_odomX_, gps_odomY_, gps_odomZ_, gps_error_);
}

void QNode::trajectoryCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    line_info_.clear();
    geometry_msgs::Point temp;

    for(int i=0; i<msg->points.size(); i++)
    {
      if(i%2 == 0) {
        temp = msg->points[i];
      }
      else {
        Line line;

        line.start_point = temp;
        line.end_point = msg->points[i];

        line.angle_rad = atan2(line.end_point.y - line.start_point.y,
                               line.end_point.x - line.start_point.x);
        line.angle_deg = line.angle_rad*RAD2DEG;

        line_info_.push_back(line);
      }
    }

    //std::cout << "Line Segment done" << std::endl;
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
            ROS_INFO("Service Result: %d", srv.response.success);
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

void QNode::sendVisibleFlag(bool flag)
{
    std_msgs::Bool msg;
    msg.data = flag;

    visible_flag_pub_.publish(msg);
}

void QNode::sendMakeFlag(double distance, double degree, int spoint, int epoint, bool flag)
{
    rviz_flag_plugin::PointArray point_array;
    rviz_flag_plugin::PointArray new_point_array;
    Line temp_line;

    //std::cout << "Line cnt: " << line_info_.size() << std::endl;

    if(line_info_.empty())
    {
      ROS_INFO("Empty path!");
      return;
    }

    std_msgs::Bool msg;
    msg.data = true;
    clear_flag_pub_.publish(msg);

    if(flag)
    {
        for(int i=0; i<line_info_.size(); i++)
        {
            if(i == 0)
            {
                temp_line = line_info_[i];

                point_array.points.push_back(temp_line.start_point);
                point_array.points.push_back(temp_line.end_point);
                continue;
            }

            Line cur_line = line_info_[i];

            double degree_diff = abs(temp_line.angle_deg - cur_line.angle_deg);
            double distance_diff = sqrt(pow(temp_line.end_point.x - cur_line.end_point.x, 2)
                                        +pow(temp_line.end_point.y - cur_line.end_point.y, 2));

            //line condition
            if( (degree_diff > degree) && (distance_diff > distance) )
            {
                point_array.points.push_back(cur_line.end_point);

                temp_line = cur_line;
            }
        }
    }
    else
    {
      for(int i=line_info_.size()-1; i>=0; i--)
      {
          if(i == (line_info_.size()-1))
          {
              temp_line = line_info_[i];

              point_array.points.push_back(temp_line.end_point);
              point_array.points.push_back(temp_line.start_point);
              continue;
          }

          Line cur_line = line_info_[i];

          double degree_diff = abs(temp_line.angle_deg - cur_line.angle_deg);
          double distance_diff = sqrt(pow(temp_line.start_point.x - cur_line.start_point.x, 2)
                                      +pow(temp_line.start_point.y - cur_line.start_point.y, 2));

          //line condition
          if( (degree_diff > degree) && (distance_diff > distance) )
          {
              point_array.points.push_back(cur_line.start_point);

              temp_line = cur_line;
          }
      }
    }

    new_point_array.points.assign(point_array.points.begin() + spoint, point_array.points.end() - epoint);
    make_flag_pub_.publish(new_point_array);
    emit pushTotalPoints(new_point_array.points.size());
}
