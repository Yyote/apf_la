#include "ros/ros.h"
#include <cstdlib>
#include "apf_la/GetTrajectory.h"
#include "apf_la/GlobalTrajectory.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"


ros::ServiceClient client;
ros::Publisher trajectory_pub;
ros::Publisher trajectory_vis_pub;


geometry_msgs::PoseStamped bufferized_goal;
std::vector<nav_msgs::Odometry> bufferized_odometry;
int succ = 0;


void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    bufferized_goal = *goal;

    apf_la::GlobalTrajectory trajectory;

    apf_la::GetTrajectory srv;
    srv.request.goal_position.pose.position.x = goal->pose.position.x;
    srv.request.goal_position.pose.position.y = goal->pose.position.y;

    ROS_INFO_STREAM("Got goal. \n");

    if (client.call(srv))
    {
        for(int i = 0; i < srv.response.path.size(); i++)
        {
            trajectory.waypoints.push_back(srv.response.path.at(i));
        }
        trajectory_pub.publish(trajectory);
        ROS_INFO_STREAM("Published trajectory with" << srv.response.path.size() << "waypoints\n");
    }
    
    else
    {
        ROS_ERROR("Failed to call service.");
    }
}


void vel_cb(const geometry_msgs::Twist::ConstPtr& velocity)
{
    if( sqrt(pow(velocity->linear.x, 2) + pow(velocity->linear.y, 2)) < 0.01 && abs(velocity->angular.z < 0.2) )
    {
        succ++;
    }
}


void odom_cb (const nav_msgs::Odometry::ConstPtr& odom)
{
    for (int i = 1; i < bufferized_odometry.size(); i++)
    {
        bufferized_odometry.at(i-1) = bufferized_odometry.at(i);
    }
    bufferized_odometry.at(bufferized_odometry.size()-1) = *odom;

    if( sqrt(pow(odom->twist.twist.linear.x, 2) + pow(odom->twist.twist.linear.y, 2)) < 0.01 && abs(odom->twist.twist.angular.z < 0.2) )
    {
        succ++;
    }

    // std::vector<geometry_msgs::Point> odom_delta;
    // odom_delta.resize(bufferized_odometry.size()-1);

    // for(int i = 1; i < bufferized_odometry.size() - 1; i++)
    // {
    //     odom_delta.at(i).x = bufferized_odometry.at(i).pose.pose.position.x - bufferized_odometry.at(i-1).pose.pose.position.x;
    //     odom_delta.at(i).y = bufferized_odometry.at(i).pose.pose.position.y - bufferized_odometry.at(i-1).pose.pose.position.y;
    // }

    // for (int i = 1; i < odom_delta.size() - 1; i++)
    // {
    //     if (sqrt(pow(odom_delta.at(i).x, 2) + pow(odom_delta.at(i).y, 2)) - sqrt(pow(odom_delta.at(i-1).x, 2) + pow(odom_delta.at(i-1).y, 2)) > 0.001)
    //     {
    //         succ++;
    //     }
    // }

    if(succ >= 25)
    {
        apf_la::GetTrajectory srv;
        srv.request.goal_position.pose.position.x = bufferized_goal.pose.position.x;
        srv.request.goal_position.pose.position.y = bufferized_goal.pose.position.y;
        ROS_ERROR_STREAM("Stuck! Resending goal...\n");

        if (client.call(srv))
        {
            apf_la::GlobalTrajectory trajectory;
            for(int i = 0; i < srv.response.path.size(); i++)
            {
                trajectory.waypoints.push_back(srv.response.path.at(i));
            }
            trajectory_pub.publish(trajectory);
            ROS_INFO_STREAM("Published trajectory with" << srv.response.path.size() << "waypoints\n");
            succ = 0;
        }

        else
        {
            ROS_ERROR("Failed to call service.");
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_client_node");

    ros::NodeHandle n;
    client = n.serviceClient<apf_la::GetTrajectory>("/astar/get_trajectory");
    ros::Subscriber goal_sub = n.subscribe("/move_base_simple/goal", 100, goal_cb);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 100, vel_cb);
    ros::Subscriber odom_sub = n.subscribe("/odom", 100, odom_cb);
    trajectory_pub = n.advertise<apf_la::GlobalTrajectory>("/astar/trajectory", 10);

    bufferized_odometry.resize(10);

    ros::spin();
    return 0;
}