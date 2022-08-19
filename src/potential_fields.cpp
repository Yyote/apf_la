// PotentialFields algorithm for lidar in PointCloud  !!2D!!
// 
// Listens to topic /transPC
// Publishes to topic /potential_field_result
// 
// frame: laser
// 
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "apf_la/VelocitiesPC.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include <cmath>
#include <dynamic_reconfigure/server.h>
#include <apf_la/reconfigure_potential_fieldsConfig.h>


// PRESETUP
bool enable_turn_vectors_to_global = 1;
bool enable_attraction = 1;
nav_msgs::Odometry global_odom;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub; // HOOK odom usage

// SUB catchPC creation
ros::Subscriber catchPC_sub;
ros::Publisher Catched_Cloud_and_PF;// PUBLISHER declaration

ros::NodeHandle* nh_pointer;

// Класс для работы с маркерами
class MarkerHandler {
    private:
    visualization_msgs::Marker marker;
    ros::Publisher marker_pub;
    ros::NodeHandle* nh_pointer;

    public:
    std::string module_name;

    MarkerHandler()
    {
        marker.header.stamp = ros::Time::now();
        module_name = "default";
    }

    // MarkerHandler(std::string pub_topic) 
    // {
    //     marker.header.stamp = ros::Time::now();
    //     marker_pub = rvizor.advertise<visualization_msgs::Marker>(pub_topic, 1000);
    //     module_name = "default";
    // }

    void _init(ros::NodeHandle* nh, std::string pub_topic)
    {
        nh_pointer = nh;
        advertise_to_topic(pub_topic);
    }

    void advertise_to_topic(std::string pub_topic)
    {
        // marker_pub = rvizor.advertise<visualization_msgs::Marker>(pub_topic, 1000);
        marker_pub = nh_pointer->advertise<visualization_msgs::Marker>(pub_topic, 1000);
    }

    void set_frame(std::string frame_id)
    {
        marker.header.frame_id = frame_id;
    }

    void set_namespace(std::string namespace_str)
    {
        marker.ns = namespace_str;
    }

    void set_id(int id)
    {
        marker.id = id;
    }

    void set_type(int enum_number)
    {
        /*
            ARROW = 0u,
            CUBE = 1u,
            SPHERE = 2u,
            CYLINDER = 3u,
            LINE_STRIP = 4u,
            LINE_LIST = 5u,
            CUBE_LIST = 6u,
            SPHERE_LIST = 7u,
            POINTS = 8u,
            TEXT_VIEW_FACING = 9u,
            MESH_RESOURCE = 10u,
            TRIANGLE_LIST = 11u,
        */
        if (enum_number == 0)
        {
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 1)
        {
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 2)
        {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 3)
        {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 4)
        {
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 5)
        {
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 6)
        {
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 7)
        {
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 8)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 9)
        {
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 10)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 11)
        {
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else ROS_ERROR_STREAM("NO SUCH POSITION IN THE LIST" << std::endl);
    }

    void set_pose(double x, double y, double z, tf2::Quaternion q)
    {
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = q.getX();
        marker.pose.orientation.y = q.getY();
        marker.pose.orientation.z = q.getZ();
        marker.pose.orientation.w = q.getW();
    }

    void set_scale(float x, float y, float z)
    {
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;
    }

    void set_color(float alpha, float red, float green, float blue)
    {
        marker.color.a = alpha;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
    }

    void publish_marker()
    {
        marker_pub.publish(marker);
    }

    void set_module_name(std::string new_name)
    {
        module_name = new_name;
    }

    void debug_info(std::string new_module_name)
    {
        set_module_name(new_module_name);
        //DEBUG rinfo
        //****************************************************************************************************
        if(1)
        {
            ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
            << "FUNCTION NAME: " << module_name << std::endl 
            << "VARIABLES: " << std::endl 
            << "frame id -->" << marker.header.frame_id << std::endl 
            << "id -->" << marker.id << std::endl 
            << "type -->" << marker.type << std::endl 
            << "pose x -->" << marker.pose.position.x << std::endl 
            << "pose y -->" << marker.pose.position.y << std::endl 
            << "pose z -->" << marker.pose.position.z << std::endl 
            << "quaternion w -->" << marker.pose.orientation.w << std::endl 
            << "scale x -->" << marker.scale.x << std::endl 
            << "scale y -->" << marker.scale.y << std::endl 
            << "scale z -->" << marker.scale.z << std::endl 
            << "color a -->" << marker.color.a << std::endl 
            << "color r -->" << marker.color.r << std::endl 
            << "color g -->" << marker.color.g << std::endl 
            << "color b -->" << marker.color.b << std::endl 
            << "_________________________________" << std::endl << "_________________________________" << std::endl);
        }
        //****************************************************************************************************
    }
};

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

// Потенциальные поля отталкивания от видимых точек на лазерскане
class PotentialFieldRepulsive {
    private:
    // CODEINFO Константы

    EulerAngles angles;
    ros::Publisher vis_arr_pub;

    MarkerHandler final_repulsion_arrow;
    MarkerHandler visibility_circle;

    int shutdown()
    {
        ROS_ERROR_STREAM(std::endl << std::endl << "terminated." << std::endl);
        ros::shutdown();
        return -1;
    }

    public:
    double MAX_RANGE; //max range to calculate
    double MIN_RANGE; //min range to calculate
    double MULTIPLICATOR; // мультипликатор

    bool PUBLISH_FINAL_REPULSION_ARROW;
    bool PUBLISH_VISIBLE_POINTS;
    bool PUBLISH_VISIBILITY_CIRCLE;

    double finvec_x;
    double finvec_y;

    PotentialFieldRepulsive(ros::NodeHandle* nh_pointer) 
    {
        MULTIPLICATOR = 0.0025;
        MIN_RANGE = 0.1;
        MAX_RANGE = 1.5;
        
        finvec_x = 0;
        finvec_y = 0;
        angles.roll = 0;
        angles.pitch = 0;
        angles.yaw = 0;     
        
        PUBLISH_FINAL_REPULSION_ARROW = 1;
        PUBLISH_VISIBLE_POINTS = 1;
        PUBLISH_VISIBILITY_CIRCLE = 1;
        
        vis_arr_pub = nh_pointer->advertise<sensor_msgs::PointCloud>("/debug_points", 1000);
        
        final_repulsion_arrow._init(nh_pointer, "/potential_field_repulsion_marker");
        visibility_circle._init(nh_pointer, "/visibility_circle_marker");
    }   

    void calculate_repulse_vector(const sensor_msgs::PointCloud::ConstPtr& catched_cloud)
    {
        sensor_msgs::PointCloud visible_points_cloud; // LOG NEW visible_points_cloud

        visible_points_cloud.points.resize(catched_cloud->points.capacity()); // LOG NEW debug cloud
        visible_points_cloud.header.stamp = ros::Time::now();
        visible_points_cloud.header.frame_id = "laser"; //FRAME

        finvec_y = 0;
        finvec_x = 0;

        // CODEINFO
        // Calculate average x and y and make a vector of retraction from them
        for(int i = 0; i < catched_cloud->points.capacity(); i++)
        {
            //************************    PSEUDO CONSTS     **************************
            double vector_length = sqrt( pow(catched_cloud->points.at(i).x, 2) + pow(catched_cloud->points.at(i).y, 2) );
            //************************************************************************

            //************************    LOCAL VARS     *****************************
            double tmpvec_x, tmpvec_y;
            //************************************************************************

            visible_points_cloud.points.at(i).x = NULL;
            visible_points_cloud.points.at(i).y = NULL;
            visible_points_cloud.points.at(i).z = NULL;

            int point_step = 1; // LOG NEW added point_step variable to skip points in such way that it won't effect the amplitude of the resulting vector

            if(i % point_step == 0)
            {
                if(!( pow(catched_cloud->points.at(i).x, 2) + pow(catched_cloud->points.at(i).y, 2) >= MAX_RANGE * MAX_RANGE || pow(catched_cloud->points.at(i).x, 2) + pow(catched_cloud->points.at(i).y, 2) <= MIN_RANGE * MIN_RANGE))  // Проверка на попадание в зону видимости
                {
//DEBUG INFO rinfo
//****************************************************************************************************
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: zero fight" << std::endl 
    << "VARIABLES: "
    << "vector length -->"  << sqrt(pow(catched_cloud->points.at(i).x, 2) + pow(catched_cloud->points.at(i).y, 2)) << std::endl
    << "MAX_RANGE -->"  << MAX_RANGE << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

                    if (catched_cloud->points.at(i).x != 0)
                    {
                        tmpvec_y = - MULTIPLICATOR * point_step * ( (1 / (vector_length) ) * catched_cloud->points.at(i).y / vector_length)/* - (MAX_RANGE / sqrt(2))*/ ; // BUG check atan limits and adjust the formula
                        tmpvec_x = - MULTIPLICATOR * point_step * ( (1 / (vector_length) ) * catched_cloud->points.at(i).x / vector_length)/* - (MAX_RANGE / sqrt(2))*/; // SOLVED change the formula for the pithagorean
                    }
                    else
                    {
                        tmpvec_y = MULTIPLICATOR * point_step * ( (1 / vector_length) * sin(atan(catched_cloud->points.at(i).x / catched_cloud->points.at(i).y))/* - (MAX_RANGE / sqrt(2))*/ ); // BUG check atan limits and adjust the formula
                        tmpvec_x = MULTIPLICATOR * point_step * ( (1 / vector_length) * cos(atan(catched_cloud->points.at(i).x / catched_cloud->points.at(i).y))/* - (MAX_RANGE / sqrt(2))*/ ); // SOLVED change the formula for the pithagorean
                    }

//DEBUG INFO rwarn
//****************************************************************************************************
if(0)
{
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: tmpvec_y check" << std::endl 
    << "VARIABLES: " << std::endl
    << "catched point y -->"  << catched_cloud->points.at(i).y << std::endl 
    << "catched point x -->"  << catched_cloud->points.at(i).x << std::endl 
    << "tmpvec_y -->"  << tmpvec_y << std::endl 
    << "sin(atan(catched_cloud->points.at(i).y / catched_cloud->points.at(i).x)) * (1 / vec_length) -->"  << (1 / vector_length) * sin(atan(catched_cloud->points.at(i).y / catched_cloud->points.at(i).x)) << std::endl 
    << "tmpvec_x -->"  << tmpvec_x << std::endl 
    << "cos(atan(catched_cloud->points.at(i).y / catched_cloud->points.at(i).x)) * (1 / vec_length) -->"  << (1 / vector_length) * cos(atan(catched_cloud->points.at(i).y / catched_cloud->points.at(i).x)) << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

                    if(catched_cloud->points.at(i).x > 0 && tmpvec_x > 0)
                    {
                        tmpvec_x = -tmpvec_x;
                    }

                    finvec_x += tmpvec_x;
                    finvec_y += tmpvec_y;

                    visible_points_cloud.points.at(i).x = catched_cloud->points.at(i).x;
                    visible_points_cloud.points.at(i).y = catched_cloud->points.at(i).y; // LOG NEW visible_points_cloud
                    visible_points_cloud.points.at(i).z = catched_cloud->points.at(i).z;
                }

                if(PUBLISH_VISIBLE_POINTS == 1)
                {
                    vis_arr_pub.publish(visible_points_cloud); //PUBLISHER vis_arr visible_points_cloud
                }

                if(0)
                {
                    double bufx, bufy; //DEBUG rerror related
                    bufx = finvec_x;
                    bufy = finvec_y;          

// DEBUG INFO rinfo
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl 
    << "VARIABLES: list of vars" << "\ncloud capacity, finvec_x and y, vector length, i:" << std::endl 
    << catched_cloud->points.capacity() << " "  << finvec_x << " " << finvec_y << " " << sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)) << " " << i << std::endl 
    << "catched_cloud->points.at(i).x --> " << catched_cloud->points.at(i).x << std::endl << "catched_cloud->points.at(i).y --> " << catched_cloud->points.at(i).y << std::endl 
    <<"_________________________________" << std::endl << "_________________________________" << std::endl);
}

                    // DEBUG rerror
                    if (finvec_x == finvec_y && finvec_x != 0)
                    {
                        ROS_ERROR_STREAM("FINVECX IS EQ TO FINVECY. terminated.");
                        shutdown();
                    }

                    if(finvec_x - bufx > 50 || finvec_y - bufy > 50) //DEBUG rerror
                    {
                        ROS_ERROR_STREAM("FINVEC IS EXCEEDING LIMITS AND AVOIDS FILTRATION --> " << std::endl << "finvec_x ---> " << finvec_x << std::endl << "finvec_y ---> " << finvec_y);
                        shutdown();
                    }
                }
            }
        }

if(0)
{
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: result vector in variables" << std::endl << "VARIABLES: list of vars:" << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "vector length -->" << sqrt(pow(finvec_x , 2) + pow(finvec_y , 2)) <<  std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);
}   

        // CODEINFO angle calculation to pass to rviz markers ---> calculate current(i) vector's angle 
        angles.yaw = (atan(finvec_y/finvec_x)/* - (M_PI) * (finvec_x > 0)*/); //BUG no vector inversion
        angles.yaw = (angles.yaw + 2 * M_PI) * (angles.yaw < - M_PI) + (angles.yaw) * (!(angles.yaw < - M_PI));

        tf2::Quaternion q;

        //IDEA make a bullshit filter for potential fields and create max passed speed constant
        q.setRPY(angles.roll, angles.pitch, angles.yaw - (M_PI) * (finvec_x > 0));
        q = q.normalize(); //SOLVED something is going with angle. cant say what, maybe its not a bug ---> in velocity calculation there was an if statement that tried to pass only > 0.001 coordiantes. It totally deleted all negative values from coordinates

// DEBUG INFO rinfo
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: angle calc" << std::endl 
    << "VARIABLES: " << std::endl 
    << "angles.yaw (atan)-->" << angles.yaw << std::endl << "finvec_y/finvec_x --> " << finvec_y/finvec_x << std::endl << "finvec_y --> " << finvec_y << std::endl << "finvec_x --> " << finvec_x << std::endl << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}

        // Если включено, вектор поворачивается в глобальную систему координат для визуализации и только. Передача финального вектора в глобальную систему идет в РЕГУЛЯТОРЕ
        if(enable_turn_vectors_to_global)
        {
            finvec_x = finvec_x * cos(-angles.yaw) - finvec_y * sin(-angles.yaw);
            finvec_y = finvec_x * sin(-angles.yaw) + finvec_y * cos(-angles.yaw);
        }

        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
        //* - * - * - * - * - * -     RVIZ      - * - */ - */ - * - * - * - * - *  //RVIZ 
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *

        final_repulsion_arrow.set_frame("laser"); //FRAME
        final_repulsion_arrow.set_namespace("speeds_namespace");
        final_repulsion_arrow.set_id(5);
        final_repulsion_arrow.set_type(0);
        final_repulsion_arrow.set_pose(0, 0, 0, q);
        final_repulsion_arrow.set_scale(-sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)), 0.02, 0.02);
        final_repulsion_arrow.set_color(1.0, 0.0, 1.0, 0.0);
        // final_repulsion_arrow.debug_info("Стрелочка");

        if(PUBLISH_FINAL_REPULSION_ARROW == 1)
        {
            final_repulsion_arrow.publish_marker(); // PUBLISHER //RVIZ publish
        }

        //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*

        tf2::Quaternion q2;
        q2.setX(0);
        q2.setY(0);
        q2.setZ(0);
        q2.setW(1);

        visibility_circle.set_frame("laser"); //FRAME
        visibility_circle.set_namespace("speeds_namespace");
        visibility_circle.set_id(2);
        visibility_circle.set_type(3);
        visibility_circle.set_pose(0, 0, -0.2, q2);
        visibility_circle.set_scale(2 * MAX_RANGE, 2 * MAX_RANGE, 0.05);
        visibility_circle.set_color(0.3, 0.0, 0.0, 1.0);

        if(PUBLISH_VISIBILITY_CIRCLE == 1)
        {
            visibility_circle.publish_marker(); // PUBLISHER //RVIZ publish
        }
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *

        // Если включено, вектор поворачивается в обратно в локальную систему координат. Передача финального вектора в глобальную систему идет в РЕГУЛЯТОРЕ
        if(enable_turn_vectors_to_global)
        {
            finvec_x = finvec_x * cos(angles.yaw) - finvec_y * sin(angles.yaw);
            finvec_y = finvec_x * sin(angles.yaw) + finvec_y * cos(angles.yaw);
        }
    }
};

// Потенциальные поля притяжения к целевой точке
class   PotentialFieldAttractive
{
    private:
    EulerAngles angles;
    double vector_length;
    double tmpvec_x = 0;
    double tmpvec_y = 0;

    public:
    double MULTIPLICATOR;
    double goal_x /*= 0*/; // Полученная координата х
    double goal_y /*= 0*/; // Полученная у
    double finvec_x; // Рассчитанная точка
    double finvec_y; // Рассчитанная точка
    MarkerHandler final_attraction_arrow;
    MarkerHandler goal;

    float MAX_VECTOR;
    bool PUBLISH_FINAL_ATTRACTION_ARROW;
    bool PUBLISH_GOAL;
    // IDEA min_vector, goal_accomplished_range;

    PotentialFieldAttractive()
    {
        MULTIPLICATOR = 0.2;   
        MAX_VECTOR = 0.3;

        goal_x = NULL;
        goal_y = NULL;
        finvec_x = 0;
        finvec_y = 0;
        
        PUBLISH_FINAL_ATTRACTION_ARROW = 1;
        PUBLISH_GOAL = 1;
        
        final_attraction_arrow._init(nh_pointer, "/potential_field_attraction_marker");
        goal._init(nh_pointer, "/goal_marker");
    }

    void calculate_attract_vector()
    {
        finvec_y = 0;
        finvec_x = 0;

        if(goal_x != NULL && goal_y != NULL)
        {
            tmpvec_x = goal_x - global_odom.pose.pose.position.x;
            tmpvec_y = goal_y - global_odom.pose.pose.position.y;
        }

        // tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w); // HOOK odom usage

        vector_length = sqrt((tmpvec_x * tmpvec_x) + (tmpvec_y * tmpvec_y));

        finvec_y = MULTIPLICATOR * ((vector_length) * (tmpvec_y / vector_length))/* - (MAX_RANGE / sqrt(2))*/; // BUG check atan limits and adjust the formula
        finvec_x = MULTIPLICATOR * ((vector_length) * (tmpvec_x / vector_length))/* - (MAX_RANGE / sqrt(2))*/; // SOLVED change the formula for the pithagorean

        tmpvec_x = finvec_x;
        tmpvec_y = finvec_y;

        tf2::Quaternion qodom;
        tf2::fromMsg(global_odom.pose.pose.orientation, qodom);
        angles.get_RPY_from_quaternion(qodom);
        // tf2::Matrix3x3 m(qodom);
        // m.getRPY(angles.roll, angles.pitch, angles.yaw);

        angles.yaw = -angles.yaw;

        finvec_x = tmpvec_x * cos(angles.yaw) - tmpvec_y * sin(angles.yaw);
        finvec_y = tmpvec_x * sin(angles.yaw) + tmpvec_y * cos(angles.yaw);

        if (finvec_x > MAX_VECTOR || finvec_x < - MAX_VECTOR)
        {
            double tmp;
            tmp = finvec_x;
            finvec_x = MAX_VECTOR * (finvec_x > MAX_VECTOR) - MAX_VECTOR * (finvec_x < MAX_VECTOR);
            tmp = finvec_x / tmp;
            finvec_y = finvec_y * tmp;
        }


        if (finvec_y > MAX_VECTOR || finvec_y < - MAX_VECTOR)
        {
            double tmp;
            tmp = finvec_y;
            finvec_y = MAX_VECTOR * (finvec_y > MAX_VECTOR) - MAX_VECTOR * (finvec_y < MAX_VECTOR);
            tmp = finvec_y / tmp;
            finvec_x = finvec_x * tmp;
        }


        // // angles.yaw = (atan(finvec_y / finvec_x)/* - (M_PI) * (finvec_x > 0)*/); //BUG no vector inversion
        // // // angles.yaw = (angles.yaw + 2 * M_PI) * (angles.yaw < - M_PI) + (angles.yaw) * (!(angles.yaw < - M_PI));
        // // angles.yaw = M_PI;


//DEBUG rinfo finvec of attraction
//****************************************************************************************************
if(0)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: finvec of attraction" << std::endl 
    << "VARIABLES: " << std::endl 
    << "finvec_x -->" << finvec_x << std::endl 
    << "finvec_y -->" << finvec_y << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

ROS_WARN_STREAM(std::endl << "angle is" << angles.yaw << std::endl);

        //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
        //------------------------------------------------------------------------------
        // VISUALIZATION //RVIZ visualiztion code // TODO make it a func
        //------------------------------------------------------------------------------

        angles.yaw = atan(finvec_y / finvec_x) - M_PI * (finvec_x < 0);

        // Если включено, вектор поворачивается в глобальную систему координат для визуализации и только. Передача финального вектора в глобальную систему идет в РЕГУЛЯТОРЕ
        if(enable_turn_vectors_to_global)
        {
            finvec_x = finvec_x * cos(-angles.yaw) - finvec_y * sin(-angles.yaw);
            finvec_y = finvec_x * sin(-angles.yaw) + finvec_y * cos(-angles.yaw);
        }

        tf2::Quaternion q;
        angles.setRPY_of_quaternion(q);
        // q.setRPY(0, 0, 0);
        q.normalize();

        final_attraction_arrow.set_frame("laser"); //FRAME
        final_attraction_arrow.set_namespace("speeds_namespace2");
        final_attraction_arrow.set_id(34);
        final_attraction_arrow.set_type(0);
        // final_attraction_arrow.set_type(2);
        // final_attraction_arrow.set_pose(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, q);
        final_attraction_arrow.set_pose(0, 0, 0, q);
        final_attraction_arrow.set_scale(sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)), 0.02, 0.02);
        // final_attraction_arrow.set_scale(0.2, 0.2, 0.2);
        final_attraction_arrow.set_color(1.0, 1.0, 1.0, 0.0);
        // final_attraction_arrow.debug_info("Arrow");

        if(PUBLISH_FINAL_ATTRACTION_ARROW == 1)
        {
            final_attraction_arrow.publish_marker(); // PUBLISHER //RVIZ publish
        }

        tf2::Quaternion q1;

        //IDEA make a bullshit filter for potential fields and create max passed speed constant
        // q1.setRPY(angles.roll, angles.pitch, angles.yaw - (M_PI) * (finvec_x > 0));
        q1.setRPY(0, 0, 0);
        q1 = q1.normalize(); //SOLVED something is going with angle. cant say what, maybe its not a bug ---> in velocity calculation there was an if statement that tried to pass only > 0.001 coordiantes. It totally deleted all negative values from coordinates

        goal.set_frame("odom"); //FRAME
        goal.set_namespace("speeds_namespace");
        goal.set_id(76);
        goal.set_type(2);
        goal.set_pose(goal_x, goal_y, 0, q1);
        goal.set_scale(0.05, 0.05, 0.05);
        goal.set_color(1.0, 1.0, 0.0, 0.0);
        // goal.debug_info("Goal");

        if(PUBLISH_GOAL == 1)
        {
            goal.publish_marker(); // PUBLISHER //RVIZ publish
        }
        //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*

        // Если включено, вектор поворачивается обратно в локальную систему координат. Передача финального вектора в глобальную систему идет в РЕГУЛЯТОРЕ
        if(enable_turn_vectors_to_global)
        {
            finvec_x = finvec_x * cos(angles.yaw) - finvec_y * sin(angles.yaw);
            finvec_y = finvec_x * sin(angles.yaw) + finvec_y * cos(angles.yaw);
        }
    }
};

// Регулятор скоростей. Принимает вектора из потенциальных полей, суммирует их и возвращает скорость
class SpeedRegulator2D
{
    private:
    double x_regulated;
    double y_regulated;
    ros::Publisher speed_pub;
    MarkerHandler total_arrow;
    ros::Publisher potential_fields_vector_pub;

    public:
    double MAX_LINEAR_SPEED;
    double MAX_ANGULAR_SPEED;
    double MAX_ANGLE_TO_ACCEPT_MOVEMENT;
    int REGULATOR_MODE; //1 - wheeled bot safe, 2 - quadrocopter, 3 - y is turning by yaw
    int repulsion_ready;
    int attraction_ready;
    int PUBLISH_TOTAL_ARROW;

    SpeedRegulator2D()
    {
        REGULATOR_MODE = 3;
        MAX_ANGLE_TO_ACCEPT_MOVEMENT = 0.1;
        PUBLISH_TOTAL_ARROW = 1;
        MAX_LINEAR_SPEED = 0.2;
        MAX_ANGULAR_SPEED = 0.75;

        x_regulated = 0;
        y_regulated = 0;
        repulsion_ready = 0;
        attraction_ready = 0;

        total_arrow._init(nh_pointer, "/total_potential_fields_arrow");
        // total_arrow.advertise_to_topic("/final_PF_arrow");
        
        speed_pub = nh_pointer->advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        potential_fields_vector_pub = nh_pointer->advertise<geometry_msgs::Vector3Stamped>("/potential_field_result_vector", 1000);
    }

    void set_regulate_one_vector(double x, double y)
    {
        x_regulated = x;
        y_regulated = y;
    }

    void set_regulate_one_vector_OVERRIDE(double &x, double &y)
    {
        x_regulated = x;
        y_regulated = y;
        x = 0;
        y = 0;
    }

    void set_regulate_one_vector_sum(double x, double y)
    {
        x_regulated += x;
        y_regulated += y;
        if (repulsion_ready == 1 && attraction_ready == 1 || REGULATOR_MODE == 2 && repulsion_ready == 1)
        {
            regulate();
        }
    }

    void set_sum_two_vectors_to_regulate_OVERRIDE(double &x1, double &y1, double &x2, double &y2)
    {
        x_regulated = x1 + x2;
        y_regulated = y1 + y2;
        x1 = 0;
        y1 = 0;
        x2 = 0;
        y2 = 0;
    }

    void set_sum_two_vectors_to_regulate_HALFOVERRIDE(double &x1, double &y1, double x2, double y2)
    {
        x_regulated = x1 + x2;
        y_regulated = y1 + y2;
        x1 = 0;
        y1 = 0;
    }

    void regulate()
    {
        EulerAngles angles;
        angles.yaw = (atan(y_regulated / x_regulated) - (M_PI) * (x_regulated < 0)); //BUG no vector inversion
        angles.yaw = (angles.yaw + 2 * M_PI) * (angles.yaw < - M_PI) + (angles.yaw) * (!(angles.yaw < - M_PI));

        EulerAngles odom_orientation;
        tf2::Quaternion qodom;
        qodom.setX(global_odom.pose.pose.orientation.x);
        qodom.setY(global_odom.pose.pose.orientation.y);
        qodom.setZ(global_odom.pose.pose.orientation.z);
        qodom.setW(global_odom.pose.pose.orientation.w);
        odom_orientation.get_RPY_from_quaternion(qodom);

//DEBUG rinfo
//****************************************************************************************************
if(1)
{
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: check attraction vector on regulation" << std::endl 
    << "VARIABLES: "<< std::endl 
    << "x -->" << x_regulated << std::endl 
    << "y -->" << y_regulated << std::endl 
    << "yaw -->" << angles.yaw << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

        if(enable_turn_vectors_to_global)
        {
            x_regulated = x_regulated * cos(-odom_orientation.yaw) - y_regulated * sin(-odom_orientation.yaw);
            y_regulated = x_regulated * sin(-odom_orientation.yaw) + y_regulated * cos(-odom_orientation.yaw);
        }

//DEBUG rinfo
//****************************************************************************************************
if(1)
{
    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: check attraction vector on regulation" << std::endl 
    << "VARIABLES: "<< std::endl 
    << "x -->" << x_regulated << std::endl 
    << "y -->" << y_regulated << std::endl 
    << "yaw -->" << angles.yaw << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************

        //CODEINFO Twist init and pub
        geometry_msgs::Twist twist;
        if (REGULATOR_MODE == 1 || REGULATOR_MODE == 2)
        {
            if(sqrt(pow(x_regulated, 2) + pow(y_regulated, 2)) < 0.01)
            {
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.angular.z = 0;
                ROS_ERROR_STREAM(std::endl << "Speed vector is too small" << std::endl);
            }
            else
            {
                if(abs(angles.yaw) - MAX_ANGLE_TO_ACCEPT_MOVEMENT < 0)
                {
                    twist.angular.z = 0;
                    twist.linear.x = x_regulated * (x_regulated < MAX_LINEAR_SPEED) + MAX_LINEAR_SPEED * (x_regulated >= MAX_LINEAR_SPEED);
                    twist.linear.y = y_regulated * (y_regulated < MAX_LINEAR_SPEED) + MAX_LINEAR_SPEED * (y_regulated >= MAX_LINEAR_SPEED);
                }

                else if (angles.yaw > 0 && angles.yaw < M_PI && REGULATOR_MODE == 1)
                {
                    // ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << y_regulated/x_regulated << std::endl << "yaw --> " << angles.yaw * 180 / M_PI << " degrees." << std::endl);
                    twist.angular.z = 0.4;
                    twist.linear.x = 0;
                    twist.linear.y = 0;
                }

                else if (angles.yaw < 0 && angles.yaw >= -M_PI && REGULATOR_MODE == 1)
                {
                    // ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << y_regulated/x_regulated << std::endl << "yaw --> " << angles.yaw * 180 / M_PI << " degrees." << std::endl);
                    twist.angular.z = -0.4;
                    twist.linear.x = 0;
                    twist.linear.y = 0;

                }
                else
                {
if(0)
{
    ROS_ERROR_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: angle calc" << std::endl 
    << "VARIABLES: " << std::endl 
    << "angles.yaw (atan)-->" << angles.yaw << std::endl << "y_regulated/x_regulated --> " << y_regulated/x_regulated << std::endl << "y_regulated --> " << y_regulated << std::endl << "x_regulated --> " << x_regulated << std::endl << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//DEBUG INFO rerror
//****************************************************************************************************
if(0)
{
    ROS_ERROR_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
    << "FUNCTION NAME: some shit is happening" << std::endl 
    << "VARIABLES: "
    << "angles.yaw -->" << angles.yaw << std::endl 
    << "REGULATOR_MODE -->" << REGULATOR_MODE << std::endl 
    << "M_PI -->" << M_PI << std::endl 
    << "_________________________________" << std::endl << "_________________________________" << std::endl);
}
//****************************************************************************************************
                }
            }
        }

        else if (REGULATOR_MODE == 3)
        {
            twist.angular.z = y_regulated * (y_regulated < MAX_ANGULAR_SPEED) + MAX_ANGULAR_SPEED * (!(y_regulated < MAX_ANGULAR_SPEED));
            twist.linear.x = x_regulated  * (x_regulated < MAX_LINEAR_SPEED) + MAX_LINEAR_SPEED * (x_regulated >= MAX_LINEAR_SPEED);
        }

        else if(REGULATOR_MODE == 4)
        {
            geometry_msgs::Vector3Stamped vector;
            
            vector.vector.x = x_regulated;
            vector.vector.y = y_regulated;
            vector.vector.z = 0;

            potential_fields_vector_pub.publish(vector);
        }

        tf2::Quaternion q_finArrow;
        q_finArrow.setRPY(angles.roll, angles.pitch, angles.yaw);
        q_finArrow.normalize();

        total_arrow.set_frame("laser"); //FRAME
        total_arrow.set_namespace("speeds_namespace2");
        total_arrow.set_id(8);
        total_arrow.set_type(0);
        total_arrow.set_pose(0, 0, 0, q_finArrow);
        total_arrow.set_scale(sqrt(pow(x_regulated, 2) + pow(y_regulated, 2)), 0.05, 0.05);
        total_arrow.set_color(1.0, 0.0, 1.0, 1.0);

        if(PUBLISH_TOTAL_ARROW == 1)
        {
            total_arrow.publish_marker(); // PUBLISHER //RVIZ publish
        }

        speed_pub.publish(twist); //PUBLISHER twist
        x_regulated = 0;
        y_regulated = 0;

        repulsion_ready = 0;
        attraction_ready = 0;
    }
};

PotentialFieldRepulsive *repulse_pointer;
SpeedRegulator2D *regulator_pointer;
PotentialFieldAttractive *attract_pointer;


// Колбэк динамической реконфигурации
void dynamic_reconfigure_callback(apf_la::reconfigure_potential_fieldsConfig &config, uint32_t level) 
{
	repulse_pointer->MULTIPLICATOR = config.repulsion_multiplier;
    repulse_pointer->MAX_RANGE = config.max_calculation_distance;
    repulse_pointer->MIN_RANGE = config.min_calculation_distance;
    repulse_pointer->PUBLISH_FINAL_REPULSION_ARROW = config.RVIZ_repulsion_arrow;
    repulse_pointer->PUBLISH_VISIBLE_POINTS = config.RVIZ_calculated_points_cloud;
    repulse_pointer->PUBLISH_VISIBILITY_CIRCLE = config.RVIZ_visibility_circle;

    attract_pointer->MULTIPLICATOR = config.attraction_multiplier;
    attract_pointer->MAX_VECTOR = config.max_vector;
    attract_pointer->PUBLISH_FINAL_ATTRACTION_ARROW = config.RVIZ_attraction_arrow;
    attract_pointer->PUBLISH_GOAL = config.RVIZ_goal;

    enable_attraction = config.enable_attraction;
    enable_turn_vectors_to_global = config.enable_turn_vectors_to_global;

    regulator_pointer->REGULATOR_MODE = config.regulator_mode;
    regulator_pointer->MAX_LINEAR_SPEED = config.max_linear_speed;
    regulator_pointer->MAX_ANGULAR_SPEED = config.max_angular_speed;
    regulator_pointer->MAX_ANGLE_TO_ACCEPT_MOVEMENT = config.max_angle_to_accept_movement;
} 

// Колбэк лазерскана
void got_scan_cb(const sensor_msgs::PointCloud::ConstPtr& catched_cloud)
{
    repulse_pointer->calculate_repulse_vector(catched_cloud);
    regulator_pointer->set_regulate_one_vector_sum(repulse_pointer->finvec_x, repulse_pointer->finvec_y);
    regulator_pointer->repulsion_ready = 1;
    // ЭТО БЫЛ РАССЧЕТ ОТТАЛКИВАНИЯ

    attract_pointer->calculate_attract_vector();
    if(enable_attraction == 1)
    {
        regulator_pointer->set_regulate_one_vector_sum(attract_pointer->finvec_x, attract_pointer->finvec_y);
    }
    regulator_pointer->attraction_ready = 1;
    // ЭТО БЫЛ РАССЧЕТ ПРИТЯГИВАНИЯ
}

// Колбэк целевой точки(2d nav goal rviz)
void got_goal_cb(const geometry_msgs::PoseStamped::ConstPtr& catched_goal)
{
    attract_pointer->goal_x = catched_goal->pose.position.x;
    attract_pointer->goal_y = catched_goal->pose.position.y; // RETHINK
}

// Колбэк одометрии
void got_odom_cb(const nav_msgs::Odometry::ConstPtr& catched_odom) // HOOK odom usage
{
    global_odom.pose.pose.position.x = catched_odom->pose.pose.position.x; // LINK IN id12-031-23 get parameter
    global_odom.pose.pose.position.y = catched_odom->pose.pose.position.y; // LINK IN id12-031-23 get parameter 
    global_odom.pose.pose.orientation.w = catched_odom->pose.pose.orientation.w;
    global_odom.pose.pose.orientation.x = catched_odom->pose.pose.orientation.x;
    global_odom.pose.pose.orientation.y = catched_odom->pose.pose.orientation.y;
    global_odom.pose.pose.orientation.z = catched_odom->pose.pose.orientation.z; 
} // RETHINK

// Мэйн функция
int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_fields");
    ros::NodeHandle nh;
    nh_pointer = &nh;

    PotentialFieldRepulsive repulse(nh_pointer);
    PotentialFieldAttractive attract;
    SpeedRegulator2D regulator;

    repulse_pointer = &repulse;
    regulator_pointer = &regulator;
    attract_pointer = &attract;

    // NODEHANDLE

    //PUBLISHER adverts
    catchPC_sub = nh.subscribe("/transPC", 10, got_scan_cb); // SUB catchPC sub

    if (enable_attraction == 1)
    {
        goal_sub = nh.subscribe("/move_base_simple/goal", 1000, got_goal_cb);
    }
    
    odom_sub = nh.subscribe("/odom", 1000, got_odom_cb); //  HOOK odom usage

    dynamic_reconfigure::Server<apf_la::reconfigure_potential_fieldsConfig> server;
    dynamic_reconfigure::Server<apf_la::reconfigure_potential_fieldsConfig>::CallbackType f;
    f = boost::bind(&dynamic_reconfigure_callback, _1, _2); 
    server.setCallback(f);

    ros::spin();

    return 0;
}   