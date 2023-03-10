#include <cmath>
#include "ros/ros.h"

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <ublox_msgs/NavPVT.h>


static ros::Subscriber odom_sub;
static ros::Publisher twist_pub;
static ros::Publisher twist_cov_pub;
static ros::Publisher pvt_pub;


static double radianToDegree(double radian)
{
    double pi = 3.14159;
    return(radian * (180 / pi));
}

static void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
    geometry_msgs::TwistStamped current_twist_msg;
    geometry_msgs::TwistWithCovarianceStamped current_twist_convariances_msg;
    ublox_msgs::NavPVT current_navpvt_msg;
    std_msgs::Header header_msg;

    header_msg.frame_id = "base_link";
    header_msg.stamp = msg->header.stamp;
    
    // twist w/o cov
    current_twist_msg.twist = msg->twist.twist;
    current_twist_msg.header = header_msg;
    twist_pub.publish(current_twist_msg);

    // twist w/ cov
    current_twist_convariances_msg.twist = msg->twist;
    current_twist_convariances_msg.header = header_msg;
    twist_cov_pub.publish(current_twist_convariances_msg);

    // pvt
    Eigen::Quaternionf quad_t(msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
        );
    quad_t.normalize();
    auto euler = quad_t.toRotationMatrix().eulerAngles(0, 1, 2);
    current_navpvt_msg.heading =  int(radianToDegree(euler[2])*100000);
    pvt_pub.publish(current_navpvt_msg);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fixposition_converter_node");
    ros::NodeHandle nh;
    odom_sub = nh.subscribe("/fixposition/odometry_enu", 10, odomCallback);
    twist_pub = nh.advertise<geometry_msgs::TwistStamped>("/localization/twist_estimator/twist", 5);
    twist_cov_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/localization/twist_estimator/twist_with_covariance", 5);
    pvt_pub = nh.advertise<ublox_msgs::NavPVT>("/fixposition/navpvt", 5);
    ros::spin();
    return 0;
}