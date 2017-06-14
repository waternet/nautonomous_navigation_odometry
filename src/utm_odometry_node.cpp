/**
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nautonomous_gps_adapter/PointStampedWithCovariance.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

ros::Publisher odom_pub;
std::string frame_id, child_frame_id, zone;

double rot_cov;

/**
 * \brief Publishes odometry to '/gps/odom'.
 * \params sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */

void publishOdometry(const nautonomous_gps_adapter::PointStampedWithCovarianceConstPtr& pointStampedWithCovariance){
    if (odom_pub) {
        nav_msgs::Odometry odom;

        odom.header.stamp = pointStampedWithCovariance->header.stamp;

        if (frame_id.empty())
        {
            odom.header.frame_id = pointStampedWithCovariance->header.frame_id;
        } 
        else 
        {
            odom.header.frame_id = frame_id;
        }

        odom.child_frame_id = child_frame_id;

        odom.pose.pose.position.x = pointStampedWithCovariance->point.x;
        odom.pose.pose.position.y = pointStampedWithCovariance->point.y;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        //ROS_INFO("Voor controle x: %f y: %f z: %f / x: %f y: %f z: %f w: %f", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

        // Use ENU covariance to build XYZRPY covariance

         boost::array<double, 36> covariance = {{
            pointStampedWithCovariance->position_covariance[0],
            pointStampedWithCovariance->position_covariance[1],
            pointStampedWithCovariance->position_covariance[2],
            0, 0, 0,
            pointStampedWithCovariance->position_covariance[3],
            pointStampedWithCovariance->position_covariance[4],
            pointStampedWithCovariance->position_covariance[5],
            0, 0, 0,
            pointStampedWithCovariance->position_covariance[6],
            pointStampedWithCovariance->position_covariance[7],
            pointStampedWithCovariance->position_covariance[8],
            0, 0, 0,
            0, 0, 0, rot_cov, 0, 0,
            0, 0, 0, 0, rot_cov, 0,
            0, 0, 0, 0, 0, rot_cov
        }};

        odom.pose.covariance = covariance;

        odom_pub.publish(odom);
    }
}

/**
 * \brief Callback function for 'fix' topic, transforms GPS to UTM using 'gps_common::LLtoUTM' func. Calls publishOdometry().
 * \param sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */

void callbackUTMFix(const nautonomous_gps_adapter::PointStampedWithCovarianceConstPtr& point) {
    publishOdometry(point);
}

/**
 * \brief Subscribes to 'fix', publishes to 'odom'.
 * \param
 * \return
 */

int main (int argc, char **argv) {
    ros::init(argc, argv, "utm_odometry_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_node("~");

    priv_node.param<std::string>("frame_id", frame_id, "");
    priv_node.param<std::string>("child_frame_id", child_frame_id, "");
    priv_node.param<double>("rot_covariance", rot_cov, 99999.0);

    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Subscriber fix_sub = node.subscribe("fix", 10, callbackUTMFix);

    ros::spin();
}
