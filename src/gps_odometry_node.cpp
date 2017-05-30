/**
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>

ros::Publisher odom_pub;
std::string frame_id, child_frame_id, zone;

double rot_cov;

double center_latitude = 0.0,   center_longitude = 0.0;    
double northing = 0.0,          easting = 0.0;
double northingCenter = 0.0,    eastingCenter = 0.0;
bool fill_initial_coord = true;
double initial_lat = 0.0, initial_lon = 0.0;

/**
 * \brief Publishes odometry to '/gps/odom'.
 * \params sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */

void publishOdometry(const sensor_msgs::NavSatFixConstPtr& fix){
    if (odom_pub) {
        nav_msgs::Odometry odom;

        odom.header.stamp = fix->header.stamp;

        if (frame_id.empty())
        {
            odom.header.frame_id = fix->header.frame_id;
        } 
        else 
        {
            odom.header.frame_id = frame_id;
        }

        odom.child_frame_id = child_frame_id;

        odom.pose.pose.position.x = easting-eastingCenter;
        odom.pose.pose.position.y = northing-northingCenter;
        odom.pose.pose.position.z = 0;

        odom.pose.pose.orientation.x = 0;
        odom.pose.pose.orientation.y = 0;
        odom.pose.pose.orientation.z = 0;
        odom.pose.pose.orientation.w = 1;

        //ROS_INFO("Voor controle x: %f y: %f z: %f / x: %f y: %f z: %f w: %f", odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);

        // Use ENU covariance to build XYZRPY covariance
        boost::array<double, 36> covariance = {{
            fix->position_covariance[0],
            fix->position_covariance[1],
            fix->position_covariance[2],
            0, 0, 0,
            fix->position_covariance[3],
            fix->position_covariance[4],
            fix->position_covariance[5],
            0, 0, 0,
            fix->position_covariance[6],
            fix->position_covariance[7],
            fix->position_covariance[8],
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
 * \brief Callback for cropper location. Sets location of center of the cropped map
 * \param 
 */
void callbackCropper(const std_msgs::Float32MultiArray& msg) {

    ROS_INFO("Callback map location: %f / %f", msg.data[0], msg.data[1]);

    center_latitude = msg.data[0];
    center_longitude = msg.data[1];
}

/**
 * \brief Callback function for 'fix' topic, transforms GPS to UTM using 'gps_common::LLtoUTM' func. Calls publishOdometry().
 * \param sensor_msgs::NavSatFixConstPtr& fix
 * \return
 */

void callbackGPSFix(const sensor_msgs::NavSatFixConstPtr& fix) {
    if(fill_initial_coord){
        initial_lat = fix->latitude;
        initial_lon = fix->longitude;
        fill_initial_coord = false;
        ROS_INFO("Initial coordinate: longitude %f, latitude %f", initial_lon, initial_lat);
    }

    if(center_latitude == 0.0 && center_longitude == 0.0){
        ROS_INFO("No specific center latitude and longitude set jet, ignoring gps callback");
        return;	
    }

    if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX && (initial_lat == fix->latitude && initial_lon == fix->longitude)) {
        ROS_INFO("No fix.");
        return;
    }

    if (fix->header.stamp == ros::Time(0)) {
        return;
    }

    gps_common::LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
    gps_common::LLtoUTM(center_latitude, center_longitude, northingCenter, eastingCenter, zone);

    publishOdometry(fix);
}

/**
 * \brief Subscribes to 'fix', publishes to 'odom'.
 * \param
 * \return
 */

int main (int argc, char **argv) {
    ros::init(argc, argv, "gps_odometry_node");
    ros::NodeHandle node;

    bool simulate;
    node.getParam("simulate", simulate);

    //Center off map is simulated or set by map cropper
    if(simulate){
        //Coenhaven
        center_latitude = 52.404434;
        center_longitude = 4.863055;
    }else{
        //Subscribe to map cropper, sets center of current map 
        ros::Subscriber crop_sub = node.subscribe("cropper_map_gps", 10, callbackCropper);        
    }  

    ros::NodeHandle priv_node("~");

    priv_node.param<std::string>("frame_id", frame_id, "");
    priv_node.param<std::string>("child_frame_id", child_frame_id, "");
    priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
    priv_node.param<double>("center_latitude", center_latitude, 0.0);
    priv_node.param<double>("center_longitude", center_longitude, 0.0);
    ROS_INFO("Center (%f, %f)", center_latitude, center_longitude);

    odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Subscriber fix_sub = node.subscribe("fix", 10, callbackGPSFix);

    ros::spin();
}
