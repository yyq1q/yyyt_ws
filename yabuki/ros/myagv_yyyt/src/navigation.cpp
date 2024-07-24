#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h> //見つからない
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "sensor_msgs/Image.h"
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <string>
#include <math.h>
#include <cstdio>
#include <string.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

class Controller
{
    public:
        Controller() : nh(), tfBuffer(), tfListener(tfBuffer)
        {
            pub_cmd_vel = nh.advertise<geometry_msgs::Twist>("cmd_vel", 5);
            timer = nh.createTimer(ros::Duration(loopRate), [&](const ros::TimerEvent& e) {
                navigation();
            });
        }

        ~Controller();

    private:
        ros::NodeHandle nh;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        ros::Timer timer;

        ros::Publisher pub_cmd_vel;

        double loopRate = 0.01;

        void navigation();
        geometry_msgs::Twist cmd_vel;
};

Controller::~Controller()
{
    cmd_vel.linear.x  = 0.0;
    cmd_vel.linear.y  = 0.0;
    cmd_vel.linear.z  = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    pub_cmd_vel.publish(cmd_vel);
}

void Controller::navigation()
{
    geometry_msgs::TransformStamped transform_base_link_to_goal;
    try
    {
        //odomから見たbase_linkの座標系
        transform_base_link_to_goal = tfBuffer.lookupTransform("base_link", "goal", ros::Time(0));

        tf2::Quaternion quaternion;
        quaternion.setW(transform_base_link_to_goal.transform.rotation.w);
        quaternion.setX(transform_base_link_to_goal.transform.rotation.x);
        quaternion.setY(transform_base_link_to_goal.transform.rotation.y);
        quaternion.setZ(transform_base_link_to_goal.transform.rotation.z);

        // クォータニオンからRPYを取得
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        double l = sqrt(transform_base_link_to_goal.transform.translation.x * transform_base_link_to_goal.transform.translation.x +
                        transform_base_link_to_goal.transform.translation.y * transform_base_link_to_goal.transform.translation.y);
        if(l > 1.0)
        {
            cmd_vel.linear.x  = transform_base_link_to_goal.transform.translation.x / l / 2.0;
            cmd_vel.linear.y  = transform_base_link_to_goal.transform.translation.y / l / 2.0;
        }
        else
        {
            cmd_vel.linear.x  = transform_base_link_to_goal.transform.translation.x / 2.0;
            cmd_vel.linear.y  = transform_base_link_to_goal.transform.translation.y / 2.0;
        }
        cmd_vel.linear.z  = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = yaw / M_PI;
        pub_cmd_vel.publish(cmd_vel);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        // cmd_vel.linear.x  = 0.0;
        // cmd_vel.linear.y  = 0.0;
        // cmd_vel.linear.z  = 0.0;
        // cmd_vel.angular.x = 0.0;
        // cmd_vel.angular.y = 0.0;
        // cmd_vel.angular.z = 0.0;
        // pub_cmd_vel.publish(cmd_vel);
        return;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation");
    Controller Controller;

    ros::spin();
}