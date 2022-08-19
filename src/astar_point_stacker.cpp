// Create a node that takes point array from astar and stack them into a vector
#include "ros/ros.h"
#include "apf_la/GlobalTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

apf_la::GlobalTrajectory trajectory_storage;

// Класс для хранения траектории
class AstarPointStacker
{
    public:
    geometry_msgs::PoseStamped actual_goal;
    float goal_threshold;
    int goal_index; 
    int goal_is_sent;

    AstarPointStacker()
    {
        goal_index = 0;
        goal_threshold = 0.1;
        goal_is_sent = 1;
    }

    // Функция для получения положения новой цели
    geometry_msgs::PoseStamped get_next_goal()
    {
        if(goal_index < trajectory_storage.waypoints.capacity())
        {
            actual_goal = trajectory_storage.waypoints.at(goal_index);
            goal_is_sent = 0;
        }

        if (goal_index < trajectory_storage.waypoints.capacity())
        {
            goal_index++;
        }
    }
};


AstarPointStacker astar_point_stacker;

ros::Publisher next_goal_pub;

void astar_cb(const apf_la::GlobalTrajectory::ConstPtr& trajectory)
{
    trajectory_storage = *trajectory;
    // for (int i = 0; i < trajectory->waypoints.capacity(); i++)
    // {
    //     // make a copy of the trajectory object in trajectory_storage
    //     trajectory_storage.waypoints.push_back(trajectory->waypoints.at(i));
    //     // trajectory_storage.waypoints.resize(trajectory->waypoints.capacity());
    //     // trajectory_storage.waypoints.at(i).header.stamp = trajectory->waypoints.at(i).header.stamp;
    //     // trajectory_storage.waypoints.at(i).header.frame_id = trajectory->waypoints.at(i).header.frame_id;
    //     // trajectory_storage.waypoints.at(i).pose.position.x = trajectory->waypoints.at(i).pose.position.x;
    //     // trajectory_storage.waypoints.at(i).pose.position.y = trajectory->waypoints.at(i).pose.position.y;
    //     // trajectory_storage.waypoints.at(i).pose.position.z = trajectory->waypoints.at(i).pose.position.z;
    //     // trajectory_storage.waypoints.at(i).pose.orientation.x = trajectory->waypoints.at(i).pose.orientation.x;
    //     // trajectory_storage.waypoints.at(i).pose.orientation.y = trajectory->waypoints.at(i).pose.orientation.y;
    //     // trajectory_storage.waypoints.at(i).pose.orientation.z = trajectory->waypoints.at(i).pose.orientation.z;
    //     // trajectory_storage.waypoints.at(i).pose.orientation.w = trajectory->waypoints.at(i).pose.orientation.w;
    //     trajectory_storage.waypoints.at(i) = trajectory->waypoints.at(i);
    // }
    ROS_INFO_STREAM("Received trajectory with " << trajectory->waypoints.capacity() << " waypoints");
    astar_point_stacker.get_next_goal();
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    if(pow(odom->pose.pose.position.x, 2) + pow(odom->pose.pose.position.y, 2) < pow(astar_point_stacker.goal_threshold, 2))
    {
        astar_point_stacker.get_next_goal();
    }
    if (astar_point_stacker.goal_is_sent == 0)
    {
        next_goal_pub.publish(astar_point_stacker.actual_goal);
        astar_point_stacker.goal_is_sent = 1;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_point_stacker");
    ros::NodeHandle n;

    ros::Subscriber astar_sub = n.subscribe<>("/astar/trajectory", 1000, astar_cb);
    ros::Subscriber odom_sub = n.subscribe<>("/odom", 1000, odom_cb);
    next_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/point_stacker/actual_goal", 1000);

    ros::spin();
    return 0;
}