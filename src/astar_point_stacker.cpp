// Create a node that takes point array from astar and stack them into a vector
#include "ros/ros.h"
#include "apf_la/GlobalTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointArray.h"
#include "geometry_msgs/Point.h"
#include "cmath"

apf_la::GlobalTrajectory trajectory_storage;
nav_msgs::Odometry global_odom;

// Класс для хранения траектории
class AstarPointStacker
{
    public:
    geometry_msgs::PoseStamped actual_goal;
    nav_msgs::OccupancyGrid own_local_map;
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
// //DEBUG rinfo
// //****************************************************************************************************
// if(1)
// {
//     ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
//     << "FUNCTION NAME: get_next_goal()" << std::endl 
//     << "VARIABLES: " << std::endl
//     << "goal_index -->" << goal_index << std::endl 
//     << "trajectory_storage.waypoints.size() -->" << trajectory_storage.waypoints.size() << std::endl 
//     << "goal_is_sent -->" << goal_is_sent << std::endl 
//     << "_________________________________" << std::endl << "_________________________________" << std::endl);
// }
// //**************************************************************************************************** 
        if(goal_index < trajectory_storage.waypoints.size())
        {
            actual_goal = trajectory_storage.waypoints.at(goal_index);
            goal_is_sent = 0;
            ROS_INFO_STREAM("1st if");
        }

        if (goal_index < trajectory_storage.waypoints.size())
        {
            goal_index++;
            ROS_INFO_STREAM("2nd if");
        }
        
    }

    // Функция для обнуления индекса
    void reset_goal_index()
    {
        goal_index = 0;
    }
};

// Класс для проверки траектории на пересечение с препятствиями в локальной карте
class AstarTrajectoryChecker
{
    public:
    apf_la::GlobalTrajectory local_trajectory;
    float discretisation_resolution;
    
    AstarTrajectoryChecker()
    {
        discretisation_resolution = 0.1;
    }

    // Функция, которая переводит траекторию в локальный фрейм 
    void localize_trajectory()
    {
        for (int i; i < local_trajectory.waypoints.size(); i++)
        {
            local_trajectory.waypoints.at(i).pose.position.x = local_trajectory.waypoints.at(i).pose.position.x - global_odom.pose.pose.position.x;
            local_trajectory.waypoints.at(i).pose.position.y = local_trajectory.waypoints.at(i).pose.position.y - global_odom.pose.pose.position.y;
        }
    }

    void discretisate_line(float x1, float y1, float x2, float y2)
    {
        geometry_msgs::PointArray normalised_point_array; // Массив точек, которые будут возвращены
        geometry_msgs::Point radius_vector; // С этим вектором мы будем работать, а по возврату все точки сместим обратно
        radius_vector.x = x2 - x1;
        radius_vector.y = y2 - y1;

        float angle_of_vector;
        angle_of_vector = (atan(radius_vector.y / radius_vector.x) - (M_PI) * (radius_vector.x < 0)); // Эти две строки кода взяты из потенциальных полей
        angle_of_vector = (angle_of_vector + 2 * M_PI) * (angle_of_vector < - M_PI) + (angle_of_vector) * (!(angle_of_vector < - M_PI));

        float discretisation_resolution_x = discretisation_resolution * cos(angle_of_vector);
        float discretisation_resolution_y = discretisation_resolution * sin(angle_of_vector);

        int number_of_points = sqrt(pow(radius_vector.x, 2) + pow(radius_vector.y, 2)) / discretisation_resolution;

        for (int i = 0; i < number_of_points; i++)
        {
            geometry_msgs::Point point;
            point.x = x1 + i * discretisation_resolution_x;
            point.y = y1 + i * discretisation_resolution_y;
            point_array.points.push_back(point);
        }
    }

    void discretisate_trajectory()
    {
        geometry_msgs::Point current_point; // Выделенная сейчас точка
        geometry_msgs::Point previous_point; // Предыдущая точка



    }

};

AstarTrajectoryChecker trajectory_checker;
AstarPointStacker astar_point_stacker;

ros::Publisher next_goal_pub;

// Колбэк астара. Получает новую траекторию и записывает ее в класс для хранения
void astar_cb(const apf_la::GlobalTrajectory::ConstPtr& trajectory)
{
    // for (int i = 0; i < trajectory->waypoints.size(); i++)
    // {
        // make a copy of the trajectory object in trajectory_storage
        // trajectory_storage.waypoints.resize(trajectory->waypoints.size());
        // trajectory_storage.waypoints.at(i).header.stamp = trajectory->waypoints.at(i).header.stamp;
        // trajectory_storage.waypoints.at(i).header.frame_id = trajectory->waypoints.at(i).header.frame_id;
        // trajectory_storage.waypoints.at(i).pose.position.x = trajectory->waypoints.at(i).pose.position.x;
        // trajectory_storage.waypoints.at(i).pose.position.y = trajectory->waypoints.at(i).pose.position.y;
        // trajectory_storage.waypoints.at(i).pose.position.z = trajectory->waypoints.at(i).pose.position.z;
        // trajectory_storage.waypoints.at(i).pose.orientation.x = trajectory->waypoints.at(i).pose.orientation.x;
        // trajectory_storage.waypoints.at(i).pose.orientation.y = trajectory->waypoints.at(i).pose.orientation.y;
        // trajectory_storage.waypoints.at(i).pose.orientation.z = trajectory->waypoints.at(i).pose.orientation.z;
        // trajectory_storage.waypoints.at(i).pose.orientation.w = trajectory->waypoints.at(i).pose.orientation.w;
        trajectory_storage.waypoints = trajectory->waypoints;
        trajectory_checker.local_trajectory.waypoints = trajectory->waypoints;

    // }
    ROS_INFO_STREAM("Received trajectory with " << trajectory->waypoints.size() << " waypoints");
    astar_point_stacker.reset_goal_index();
    astar_point_stacker.get_next_goal();
}


void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
// //DEBUG rinfo
// //****************************************************************************************************
// if(0)
// {
//     ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
//     << "FUNCTION NAME: odom_cb" << std::endl 
//     << "VARIABLES: " << std::endl 
//     << "odom.x -->" << odom->pose.pose.position.x << std::endl 
//     << "odom.y -->" << odom->pose.pose.position.y << std::endl 
//     << "odom.z -->" << odom->pose.pose.position.z << std::endl 
//     << "odom.orientation.x -->" << odom->pose.pose.orientation.x << std::endl 
//     << "odom.orientation.y -->" << odom->pose.pose.orientation.y << std::endl 
//     << "odom.orientation.z -->" << odom->pose.pose.orientation.z << std::endl 
//     << "odom.orientation.w -->" << odom->pose.pose.orientation.w << std::endl 
//     << "_________________________________" << std::endl << "_________________________________" << std::endl);
// }
// //****************************************************************************************************

// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	odom coords:" <<	odom->pose.pose.position.x << " " << odom->pose.pose.position.y <<  " " << odom->pose.pose.position.z	<<	"\n########	goal coords:" << astar_point_stacker.actual_goal.pose.position.x << " "	<< astar_point_stacker.actual_goal.pose.position.y << " " << astar_point_stacker.actual_goal.pose.position.z	<<	"\n#############################################################\n#############################################################");
// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	distance to goal:" << 	pow(odom->pose.pose.position.x - astar_point_stacker.actual_goal.pose.position.x, 2) + pow(odom->pose.pose.position.y - astar_point_stacker.actual_goal.pose.position.y, 2)	<< 		"\n########	threshold:" << 	pow(astar_point_stacker.goal_threshold, 2)	<< 		"\n#############################################################\n#############################################################");
    global_odom = *odom;

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


void local_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& local_map)
{
    astar_point_stacker.own_local_map = *local_map;
    ROS_INFO_STREAM("Received local map");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_point_stacker");
    ros::NodeHandle n;

    ros::Subscriber astar_sub = n.subscribe("/astar/trajectory", 1000, astar_cb);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_cb);
    ros::Subscriber local_map_sub = n.subscribe("/local_map", 1000, local_map_cb);
    next_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/point_stacker/actual_goal", 1000);

    ros::spin();
    return 0;
}