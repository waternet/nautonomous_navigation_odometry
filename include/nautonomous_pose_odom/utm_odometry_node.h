// ROS
#include <ros/ros.h>

// Messages
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

// Custom message
#include <nautonomous_pose_msgs/PointWithCovarianceStamped.h>

// Odom publisher
ros::Publisher odom_pub;

// Params
std::string frame_id;
std::string child_frame_id;
double rot_cov;

bool old_time;

/**
 * \brief Callback on the fix.
 * \params sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */
void callbackUTMFix(const nautonomous_pose_msgs::PointWithCovarianceStampedConstPtr& pointWithCovarianceStamped);

/**
 * \brief Subscribes to 'fix', publishes to 'odom'.
 * \param
 * \return
 */
int main (int argc, char **argv);
