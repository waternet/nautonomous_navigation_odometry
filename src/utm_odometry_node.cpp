#include <nautonomous_pose_odom/utm_odometry_node.h>

/**
 * \brief Publishes odometry to '/gps/odom'.
 * \params sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */
void callbackUTMFix(const nautonomous_pose_msgs::PointWithCovarianceStampedConstPtr& pointWithCovarianceStamped)
{
	if (odom_pub) 
	{
		nav_msgs::Odometry odom;
		odom.header.stamp = pointWithCovarianceStamped->header.stamp;

		if (old_time)
		{
			odom.header.stamp = ros::Time::now();
		}

		if (frame_id.empty())
		{
		    odom.header.frame_id = pointWithCovarianceStamped->header.frame_id;
		} 
		else 
		{
		    odom.header.frame_id = frame_id;
		}

		odom.child_frame_id = child_frame_id;

		odom.pose.pose.position.x = pointWithCovarianceStamped->point.x;
		odom.pose.pose.position.y = pointWithCovarianceStamped->point.y;
		odom.pose.pose.position.z = 0;

		odom.pose.pose.orientation.x = 0;
		odom.pose.pose.orientation.y = 0;
		odom.pose.pose.orientation.z = 0;
		odom.pose.pose.orientation.w = 1;

		// Use ENU covariance to build XYZRPY covariance

		 boost::array<double, 36> covariance = {{
		    pointWithCovarianceStamped->position_covariance[0],
		    pointWithCovarianceStamped->position_covariance[1],
		    pointWithCovarianceStamped->position_covariance[2],
		    0, 0, 0,
		    pointWithCovarianceStamped->position_covariance[3],
		    pointWithCovarianceStamped->position_covariance[4],
		    pointWithCovarianceStamped->position_covariance[5],
		    0, 0, 0,
		    pointWithCovarianceStamped->position_covariance[6],
		    pointWithCovarianceStamped->position_covariance[7],
		    pointWithCovarianceStamped->position_covariance[8],
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
 * \brief Subscribes to 'fix', publishes to 'odom'.
 * \param
 * \return
 */

int main (int argc, char **argv) 
{
    ros::init(argc, argv, "utm_odometry_node");

    ros::NodeHandle node;

    ros::NodeHandle priv_node("~");
    priv_node.param<std::string>("frame_id", frame_id, "");
    priv_node.param<std::string>("child_frame_id", child_frame_id, "");
    priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
	priv_node.param<bool>("old_time", old_time, false);

    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Subscriber fix_sub = node.subscribe("fix", 10, callbackUTMFix);

    ros::spin();
}
