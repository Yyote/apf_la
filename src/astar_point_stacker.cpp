// Create a node that takes point array from astar and stack them into a vector
#include "ros/ros.h"
#include "apf_la/GlobalTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"

apf_la::GlobalTrajectory trajectory_storage;
nav_msgs::Odometry global_odom;
double timer;

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
        goal_threshold = 0.5;
        goal_is_sent = 1;
    }

    // Функция для получения положения новой цели
    void get_next_goal()
    {
//DEBUG rinfo
//****************************************************************************************************
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: get_next_goal()" << std::endl 
    << "VARIABLES: " << std::endl
    << "goal_index -->" << goal_index << std::endl 
    << "trajectory_storage.waypoints.size() -->" << trajectory_storage.waypoints.size() << std::endl 
    << "goal_is_sent -->" << goal_is_sent << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//**************************************************************************************************** 
        if(goal_index < trajectory_storage.waypoints.size())
        {
            actual_goal = trajectory_storage.waypoints.at(goal_index);
            goal_is_sent = 0;
            ROS_INFO_STREAM("1st if");
            if (goal_index == 0)
            {
                timer = ros::Time::now().toSec();
                ROS_WARN_STREAM("Started timer ---> " << timer);
            }
        }

        if (goal_index < trajectory_storage.waypoints.size())
        {
            goal_index++;
            ROS_INFO_STREAM("2nd if");
        }
        else if (goal_index == trajectory_storage.waypoints.size() && sqrt(pow(global_odom.pose.pose.position.x, 2) + pow(global_odom.pose.pose.position.y, 2)) < 0.3)
        {
            goal_index = trajectory_storage.waypoints.size();
            timer = ros::Time::now().toSec() - timer;
            ROS_INFO_STREAM("Timer stopped ---> " << timer);    
        }
        
    }

    // Функция для обнуления индекса
    void reset_goal_index()
    {
        goal_index = 0;
    }
};


AstarPointStacker astar_point_stacker;

ros::Publisher next_goal_pub;

// Колбэк астара. Получает новую траекторию и записывает ее в класс для хранения
void astar_cb(const apf_la::GlobalTrajectory::ConstPtr& trajectory)
{
    for (int i = 0; i < trajectory->waypoints.size(); i++)
    {
        // make a copy of the trajectory object in trajectory_storage
        trajectory_storage.waypoints.resize(trajectory->waypoints.size());
        trajectory_storage.waypoints.at(i).header.stamp = trajectory->waypoints.at(i).header.stamp;
        trajectory_storage.waypoints.at(i).header.frame_id = trajectory->waypoints.at(i).header.frame_id;
        trajectory_storage.waypoints.at(i).pose.position.x = trajectory->waypoints.at(i).pose.position.x;
        trajectory_storage.waypoints.at(i).pose.position.y = trajectory->waypoints.at(i).pose.position.y;
        trajectory_storage.waypoints.at(i).pose.position.z = trajectory->waypoints.at(i).pose.position.z;
        trajectory_storage.waypoints.at(i).pose.orientation.x = trajectory->waypoints.at(i).pose.orientation.x;
        trajectory_storage.waypoints.at(i).pose.orientation.y = trajectory->waypoints.at(i).pose.orientation.y;
        trajectory_storage.waypoints.at(i).pose.orientation.z = trajectory->waypoints.at(i).pose.orientation.z;
        trajectory_storage.waypoints.at(i).pose.orientation.w = trajectory->waypoints.at(i).pose.orientation.w;
    }
    ROS_INFO_STREAM("Received trajectory with " << trajectory->waypoints.size() << " waypoints");
    astar_point_stacker.reset_goal_index();
    astar_point_stacker.get_next_goal();
}


void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    global_odom = *odom;

//DEBUG rinfo
//****************************************************************************************************
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: odom_cb" << std::endl 
    << "VARIABLES: " << std::endl 
    << "odom.x -->" << odom->pose.pose.position.x << std::endl 
    << "odom.y -->" << odom->pose.pose.position.y << std::endl 
    << "odom.z -->" << odom->pose.pose.position.z << std::endl 
    << "odom.orientation.x -->" << odom->pose.pose.orientation.x << std::endl 
    << "odom.orientation.y -->" << odom->pose.pose.orientation.y << std::endl 
    << "odom.orientation.z -->" << odom->pose.pose.orientation.z << std::endl 
    << "odom.orientation.w -->" << odom->pose.pose.orientation.w << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	odom coords:" <<	odom->pose.pose.position.x << " " << odom->pose.pose.position.y <<  " " << odom->pose.pose.position.z	<<	"\n########	goal coords:" << astar_point_stacker.actual_goal.pose.position.x << " "	<< astar_point_stacker.actual_goal.pose.position.y << " " << astar_point_stacker.actual_goal.pose.position.z	<<	"\n#############################################################\n#############################################################");
// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	distance to goal:" << 	pow(odom->pose.pose.position.x - astar_point_stacker.actual_goal.pose.position.x, 2) + pow(odom->pose.pose.position.y - astar_point_stacker.actual_goal.pose.position.y, 2)	<< 		"\n########	threshold:" << 	pow(astar_point_stacker.goal_threshold, 2)	<< 		"\n#############################################################\n#############################################################");

    if(pow(odom->pose.pose.position.x - astar_point_stacker.actual_goal.pose.position.x, 2) + pow(odom->pose.pose.position.y - astar_point_stacker.actual_goal.pose.position.y, 2) < pow(astar_point_stacker.goal_threshold, 2))
    {
        ROS_WARN_STREAM("Debug flag 1");
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