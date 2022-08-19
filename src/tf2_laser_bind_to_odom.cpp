#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::OdometryPtr& odom){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "laser";
    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;   

    tf2::Quaternion q;
    q.setX(odom->pose.pose.orientation.x);
    q.setY(odom->pose.pose.orientation.y);
    q.setZ(odom->pose.pose.orientation.z);
    q.setW(odom->pose.pose.orientation.w);  

    tf2::Quaternion qworld;
    qworld.setX(0);
    qworld.setY(0);
    qworld.setZ(0);
    qworld.setW(-1);    

    tf2::Quaternion qrotation;
    qrotation = q * qworld;   

    transformStamped.transform.rotation.x = qrotation.x();
    transformStamped.transform.rotation.y = qrotation.y();
    transformStamped.transform.rotation.w = qrotation.w();
    transformStamped.transform.rotation.z = qrotation.z();

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf2_broadcaster");    
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/odom", 10, &poseCallback);

    ros::spin();
    return 0;
};
