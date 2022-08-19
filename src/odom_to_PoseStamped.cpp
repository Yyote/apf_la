#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pose_pub;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = odom->header.stamp;
    pose.pose.orientation.w = odom->pose.pose.orientation.w;
    pose.pose.orientation.x = odom->pose.pose.orientation.x;
    pose.pose.orientation.y = odom->pose.pose.orientation.y;
    pose.pose.orientation.z = odom->pose.pose.orientation.z;
    pose.pose.position.x = odom->pose.pose.position.x;
    pose.pose.position.y = odom->pose.pose.position.y;
    pose.pose.position.z = odom->pose.pose.position.z;
    pose_pub.publish(pose);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_PoseStamped_node");
    
    ros::NodeHandle n;
    pose_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1000);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_callback);
    
    ros::spin();

    return 0;
}