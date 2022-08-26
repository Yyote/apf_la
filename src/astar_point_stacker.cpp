// Create a node that takes point array from astar and stack them into a vector
#include "ros/ros.h"
#include "apf_la/GlobalTrajectory.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Point32.h"
#include "apf_la/PointArray.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "cmath"

apf_la::GlobalTrajectory trajectory_storage;
nav_msgs::Odometry global_odom;
ros::Publisher discrete_trajectory_pub;


// Класс для работы с Эйлеровыми углами
class EulerAngles {
    public:
    EulerAngles()
    {
        roll = 0;
        pitch = 0;
        yaw = 0;
    }

    double roll;
    double pitch;
    double yaw;

    // Функция для установки Эйлеровых углов по переданным параметрам
    void setRPY(float new_roll, float new_pitch, float new_yaw)
    {
        roll = new_roll;
        pitch = new_pitch;
        yaw = new_yaw;
    }

    // Функция для передачи Эйлеровых углов в вектор вращения
    void setRPY_of_quaternion(tf2::Quaternion &q)
    {
        q.setRPY(roll, pitch, yaw);
    }

    // Функция для получения Эйлеровых углов из вектора вращения 
    void get_RPY_from_quaternion(tf2::Quaternion q)
    {
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }
};


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
    apf_la::GlobalTrajectory discrete_trajectory;
    nav_msgs::OccupancyGrid own_local_map;
    sensor_msgs::PointCloud discrete_trajectory_points_pc;
    float discretisation_resolution;
    
    AstarTrajectoryChecker()
    {
        discretisation_resolution = 0.1;
        discrete_trajectory_points_pc.header.frame_id = "base_link";
        // discrete_trajectory_points_pc.header.stamp = ros::Time::now();
        // discrete_trajectory_points_pc.points.resize(0);
    }


    // Функция, которая переводит траекторию в локальный фрейм 
    void localize_trajectory()
    {
        ROS_ERROR_STREAM("ENTERED LOCALIZE TRAJECTORY");
        for (int i = 0; i < discrete_trajectory.waypoints.size(); i++)
        {
            //     ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	traj:" << 	local_trajectory.waypoints.at(i).pose.position.x	<< " " <<  local_trajectory.waypoints.at(i).pose.position.y	 <<	"\n########	localized traj:" << 	local_trajectory.waypoints.at(i).pose.position.x - global_odom.pose.pose.position.x	<< " " << local_trajectory.waypoints.at(i).pose.position.y - global_odom.pose.pose.position.y <<		"\n#############################################################\n#############################################################");
            //     local_trajectory.waypoints.at(i).pose.position.x = local_trajectory.waypoints.at(i).pose.position.x - global_odom.pose.pose.position.x;
            //     local_trajectory.waypoints.at(i).pose.position.y = local_trajectory.waypoints.at(i).pose.position.y - global_odom.pose.pose.position.y;
            ROS_INFO_STREAM("Discrete trajectory waypoint before localizing: " << discrete_trajectory.waypoints.at(i).pose.position.x << " " << discrete_trajectory.waypoints.at(i).pose.position.y);
            discrete_trajectory.waypoints.at(i).pose.position.x = discrete_trajectory.waypoints.at(i).pose.position.x + global_odom.pose.pose.position.x;
            discrete_trajectory.waypoints.at(i).pose.position.y = discrete_trajectory.waypoints.at(i).pose.position.y + global_odom.pose.pose.position.y;
            
            // turn the point by the angle of the robot using the rotation matrix
            EulerAngles angles;
            angles.get_RPY_from_quaternion(tf2::Quaternion(global_odom.pose.pose.orientation.x, global_odom.pose.pose.orientation.y, global_odom.pose.pose.orientation.z, global_odom.pose.pose.orientation.w));


            ROS_INFO_STREAM("Global_odom yaw angle:" << angles.yaw * 180 / M_PI);

            // discrete_trajectory.waypoints.at(i).pose.position.x = discrete_trajectory.waypoints.at(i).pose.position.x * cos(angles.yaw) - discrete_trajectory.waypoints.at(i).pose.position.y * sin(angles.yaw);
            // discrete_trajectory.waypoints.at(i).pose.position.y = discrete_trajectory.waypoints.at(i).pose.position.x * sin(angles.yaw) + discrete_trajectory.waypoints.at(i).pose.position.y * cos(angles.yaw);

// BUG игрек в одной из точек считается неправильно // BUG скорее всего дискретизация просиходит неправильно
            ROS_INFO_STREAM("Discrete trajectory waypoint after localizing: " << discrete_trajectory.waypoints.at(i).pose.position.x << " " << discrete_trajectory.waypoints.at(i).pose.position.y);
        }
    }

    // Функция, которая делит линию на отрезки
    apf_la::PointArray discretisate_line(float x1, float y1, float x2, float y2)
    {
        apf_la::PointArray normalised_point_array; // Массив точек, которые будут возвращены

        int number_of_points = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2)) / discretisation_resolution;

        float discretisation_resolution_x = (x2 - x1) / number_of_points;
        float discretisation_resolution_y = (y2 - y1) / number_of_points;

        geometry_msgs::Point point;
        point.x = x1;
        point.y = y1;
        ROS_INFO_STREAM("point.x = " << point.x << " point.y = " << point.y);

        normalised_point_array.points.push_back(point);

        for (int i = 0; i < number_of_points - 1; i++) // abs(x2 - point.x) <= discretisation_precision && abs(y2 - point.y)  <= discretisation_precision
        {
            point.x = point.x + discretisation_resolution_x;
            point.y = point.y + discretisation_resolution_y;
            ROS_INFO_STREAM("point.x = " << point.x << " point.y = " << point.y);
            normalised_point_array.points.push_back(point);
        }

        // ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	x2, y2:" << 	x2  << " " << y2	<< 		"\n########	Last point:" <<   /*эта конструкция означает последнюю точку в векторе*/ 	normalised_point_array.points.at(normalised_point_array.points.size() - 1).x << " " << normalised_point_array.points.at(normalised_point_array.points.size() - 1).y	<< 		"\n#############################################################\n#############################################################");

        return normalised_point_array;
    }

    // Функция, которая делит траекторию на мелкие отрезки
    void discretisate_trajectory()
    {
        geometry_msgs::Point current_point; // Выделенная сейчас точка
        geometry_msgs::Point previous_point; // Предыдущая точка
        discrete_trajectory_points_pc.header.stamp = ros::Time::now();


        // current_point = local_trajectory.waypoints.at(1).pose.position;
        // previous_point = local_trajectory.waypoints.at(0).pose.position;
        std::vector<apf_la::PointArray> discrete_lines_storage;

        for (int i = 1; i < local_trajectory.waypoints.size() - 1; i++)
        {
            current_point = local_trajectory.waypoints.at(i).pose.position;
            previous_point = local_trajectory.waypoints.at(i - 1).pose.position;
            discrete_lines_storage.push_back(discretisate_line(previous_point.x, previous_point.y, current_point.x, current_point.y));
            // ROS_INFO_STREAM("discrete_lines_storage.size() = " << discrete_lines_storage.size());
        }

        for (int i = 0; i < discrete_lines_storage.size(); i++)
        {
            for (int j = 0; j < discrete_lines_storage.at(i).points.size(); j++)
            {
                geometry_msgs::PoseStamped pose;
                geometry_msgs::Point32 point;
                pose.pose.position = discrete_lines_storage.at(i).points.at(j);
                discrete_trajectory.waypoints.push_back(pose);
                point.x = discrete_lines_storage.at(i).points.at(j).x;
                point.y = discrete_lines_storage.at(i).points.at(j).y;
                discrete_trajectory_points_pc.points.push_back(point);
                // ROS_INFO_STREAM("point.x = " << point.x << " point.y = " << point.y);
            }
        }
        
        localize_trajectory();
        discrete_trajectory_pub.publish(discrete_trajectory_points_pc);
    }

    void clean_discrete_trajectory()
    {
        discrete_trajectory.waypoints.resize(0);
    }


    void check_trajectory()
    {
        // localize_trajectory();
        clean_discrete_trajectory();
        discretisate_trajectory();
    }
};

AstarTrajectoryChecker trajectory_checker;
AstarPointStacker point_stacker;

ros::Publisher next_goal_pub;

// Колбэк астара. Получает новую траекторию и записывает ее в класс для хранения
void astar_cb(const apf_la::GlobalTrajectory::ConstPtr& trajectory)
{
    trajectory_storage.waypoints = trajectory->waypoints;
    trajectory_checker.local_trajectory.waypoints = trajectory->waypoints;

    ROS_INFO_STREAM("Received trajectory with " << trajectory->waypoints.size() << " waypoints");
    point_stacker.reset_goal_index();
    point_stacker.get_next_goal();
    trajectory_checker.check_trajectory();
}


void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	odom coords:" <<	odom->pose.pose.position.x << " " << odom->pose.pose.position.y <<  " " << odom->pose.pose.position.z	<<	"\n########	goal coords:" << point_stacker.actual_goal.pose.position.x << " "	<< point_stacker.actual_goal.pose.position.y << " " << point_stacker.actual_goal.pose.position.z	<<	"\n#############################################################\n#############################################################");
// ROS_INFO_STREAM("\n#############################################################\n#############################################################\n########	distance to goal:" << 	pow(odom->pose.pose.position.x - point_stacker.actual_goal.pose.position.x, 2) + pow(odom->pose.pose.position.y - point_stacker.actual_goal.pose.position.y, 2)	<< 		"\n########	threshold:" << 	pow(point_stacker.goal_threshold, 2)	<< 		"\n#############################################################\n#############################################################");
    global_odom = *odom;

    if(pow(odom->pose.pose.position.x - point_stacker.actual_goal.pose.position.x, 2) + pow(odom->pose.pose.position.y - point_stacker.actual_goal.pose.position.y, 2) < pow(point_stacker.goal_threshold, 2))
    {
        // ROS_WARN_STREAM("Debug flag 1");
        point_stacker.get_next_goal();
    }

    if (point_stacker.goal_is_sent == 0)
    {
        next_goal_pub.publish(point_stacker.actual_goal);
        point_stacker.goal_is_sent = 1;
    }

// EulerAngles angles;
// angles.get_RPY_from_quaternion(tf2::Quaternion(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w));

// ROS_INFO_STREAM("Odom angles are: " << angles.roll * 180 / M_PI << " " << angles.pitch * 180 / M_PI << " " << angles.yaw * 180 / M_PI);

// angles.get_RPY_from_quaternion(tf2::Quaternion(global_odom.pose.pose.orientation.x, global_odom.pose.pose.orientation.y, global_odom.pose.pose.orientation.z, global_odom.pose.pose.orientation.w));

// ROS_INFO_STREAM("Global odom angles are: " << angles.roll * 180 / M_PI << " " << angles.pitch * 180 / M_PI << " " << angles.yaw * 180 / M_PI);
}


void local_map_cb(const nav_msgs::OccupancyGrid::ConstPtr& local_map)
{
    trajectory_checker.own_local_map = *local_map;
    // ROS_INFO_STREAM("Received local map");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_stacker");
    ros::NodeHandle n;

    ros::Subscriber astar_sub = n.subscribe("/astar/trajectory", 1000, astar_cb);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1000, odom_cb);
    ros::Subscriber local_map_sub = n.subscribe("/local_map", 1000, local_map_cb);
    next_goal_pub = n.advertise<geometry_msgs::PoseStamped>("/point_stacker/actual_goal", 1000);
    discrete_trajectory_pub = n.advertise<sensor_msgs::PointCloud>("/point_stacker/discrete_trajectory_pc", 1000);

    trajectory_checker.discretisate_line(1, 1, 1, -1);

    ros::spin();
    return 0;
}