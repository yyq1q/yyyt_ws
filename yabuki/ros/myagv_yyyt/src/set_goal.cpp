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
#include <queue>
#include <stack>

class Setting_start_goal
{
    public:
        Setting_start_goal() : nh(), tfBuffer(), tfListener(tfBuffer)
        {
            OpenAndReadCSV(csvFilePath);
            //broadcast_goalPos_tf(parent_id, child_id);

            sub_mode_msg = nh.subscribe("mode_msgs", 1, &Setting_start_goal::callback_mode, this); 

            timer = nh.createTimer(ros::Duration(loopRate), [&](const ros::TimerEvent& e) {
                // if(!setCheckpoints)
                // {
                //     set_tf_checkpoints();
                // }
                // else
                // {
                //     updateCheckpoint();
                //     broadcast_goalPos_tf_byCheckpoint(currentCheckpoint, parent_id, child_id);
                // }
                if(!saveFlag)
                {
                    updateCheckpoint2();
                }
            });

            timer2 = nh.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent& e) {
                if(mode_msg == "Go")
                {
                    saveFlag = false;
                }

                if(saveFlag)
                {
                    saveTrajectory();
                }
                else
                {
                    updateCheckpoint2();
                    followTrajectory();
                }
            });
        }

        ~Setting_start_goal();

    private:
        ros::NodeHandle nh;

        ros::Subscriber sub_mode_msg;

        tf2_ros::StaticTransformBroadcaster static_br;

        geometry_msgs::Point start_position;
        geometry_msgs::Point  goal_position;
        geometry_msgs::Quaternion start_orientation;
        geometry_msgs::Quaternion  goal_orientation;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener;
        ros::Timer timer;
        ros::Timer timer2;

        double loopRate = 0.01;

        //tfをブロードキャストする
        void broadcast_goalPos_tf(std::string parent_id, std::string child_id);
        void broadcast_goalPos_tf_byCheckpoint(int num, std::string parent_id, std::string child_id);
        std::string parent_id = "odom";
        std::string child_id  = "goal";

        //csvファイル系統の関数
        //std::string csvFilePath = "/home/er/yyyt_ws/src/myagv_yyyt/config/distination/distination.csv";
        std::string csvFilePath = "/home/yabu/yyyt_ws/src/myagv_yyyt/config/distination/distination.csv";
        std::vector<std::vector<double>> checkpoints_position;
        void OpenAndReadCSV(std::string pathToCsvFile);
        void closeCSV(std::string pathToCsvFile);

        void set_tf_checkpoints();
        bool setCheckpoints = false;

        void updateCheckpoint();
        void updateCheckpoint2();
        int currentCheckpoint = 0;
        ros::Time startTime;
        ros::Time endTime;

        bool saveFlag = true;
        void saveTrajectory();
        //std::vector<std::vector<double>> trajectory;
        std::stack<geometry_msgs::TransformStamped> trajectory;
        void followTrajectory();
        unsigned int currentTrajectory = 0;

        void callback_mode(const std_msgs::String::ConstPtr &msg);
        std::string mode_msg = "";
};

Setting_start_goal::~Setting_start_goal()
{
    closeCSV(csvFilePath);
}

void Setting_start_goal::broadcast_goalPos_tf(std::string parent_id, std::string child_id)
{
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_id;
    transformStamped.child_frame_id  = child_id;
    transformStamped.transform.translation.x = 5.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x    = 0.0;
    transformStamped.transform.rotation.y    = 0.0;
    transformStamped.transform.rotation.z    = 0.0;
    transformStamped.transform.rotation.w    = 1.0;
    static_br.sendTransform(transformStamped);
}

void Setting_start_goal::broadcast_goalPos_tf_byCheckpoint(int num, std::string parent_id, std::string child_id)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform(parent_id, "checkpoint_" + std::to_string(num), ros::Time(0));
        transformStamped.header.frame_id = parent_id;
        transformStamped.child_frame_id  = child_id;
        static_br.sendTransform(transformStamped);
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void Setting_start_goal::updateCheckpoint()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("base_link", "goal", ros::Time(0));
        double l = sqrt(
                   transformStamped.transform.translation.x * transformStamped.transform.translation.x
                 + transformStamped.transform.translation.y * transformStamped.transform.translation.y
                 );
        if(l < 0.1)
        {
            endTime = ros::Time::now();
            ros::Duration duration = endTime -startTime;
            if(duration.toSec() > 1.0)
            {
                trajectory.pop();
                currentCheckpoint = (currentCheckpoint + 1) % checkpoints_position.size();
            }
        }
        else
        {
            startTime = ros::Time::now();
        }
    }
    catch (tf2::TransformException& ex)
    {
        startTime = ros::Time::now();
        ROS_WARN("%s", ex.what());
        return;
    }
}

void Setting_start_goal::updateCheckpoint2()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("base_link", "goal", ros::Time(0));
        double l = sqrt(
                   transformStamped.transform.translation.x * transformStamped.transform.translation.x
                 + transformStamped.transform.translation.y * transformStamped.transform.translation.y
                 );
        if(l < 0.1)
        {
            trajectory.pop();
        }
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void Setting_start_goal::saveTrajectory()
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id  = "goal";
        trajectory.push(transformStamped);
        ROS_DEBUG("Saved transform to trajectory. Queue size: %zu", trajectory.size());
    }
    catch (tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
    }
}

void Setting_start_goal::followTrajectory()
{
    if(!trajectory.empty())
    {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped = trajectory.top();
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id  = "goal";
        static_br.sendTransform(transformStamped);
    }
    else
    {
        // geometry_msgs::TransformStamped transformStamped;
        // try
        // {
        //     transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
        //     transformStamped.header.stamp  =ros::Time::now();
        //     transformStamped.header.frame_id = "odom";
        //     transformStamped.child_frame_id  = "goal";
        //     static_br.sendTransform(transformStamped);
        // }
        // catch (tf2::TransformException& ex)
        // {
        //     ROS_WARN("%s", ex.what());
        //     return;
        // }
    }
}

void Setting_start_goal::OpenAndReadCSV(std::string pathToCsvFile)
{
    std::ifstream csvFile(pathToCsvFile);
    if (!csvFile.is_open()) {
        ROS_ERROR("Couldn't open CSV File : %s", pathToCsvFile.c_str());
        return;
    }
    std::string line;
    while (std::getline(csvFile, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        std::vector<double> rowData;
        // カンマで区切られた各セルを取得し、doubleに変換してベクターに追加
        while (std::getline(lineStream, cell, ',')) {
            rowData.push_back(std::stod(cell));
        }
        if(rowData.size() < 3){
            rowData.push_back(0.0);
        }
        checkpoints_position.push_back(rowData);
    }

    for(int i = 0; i < checkpoints_position.size(); i++)
    {
        for(int j = 0; j < checkpoints_position[i].size(); j++)
        {
            std::cout << checkpoints_position[i][j] << ",";
        }
        std::cout << std::endl;
    }
}

void Setting_start_goal::closeCSV(std::string pathToCsvFile)
{
    std::ifstream csvFile(pathToCsvFile);
    csvFile.close();
}

void Setting_start_goal::set_tf_checkpoints()
{
    geometry_msgs::TransformStamped transform_odom_to_base_link;
    try
    {
        //odomから見たbase_linkの座標系
        transform_odom_to_base_link = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));

        tf2::Quaternion quaternion;
        quaternion.setW(transform_odom_to_base_link.transform.rotation.w);
        quaternion.setX(transform_odom_to_base_link.transform.rotation.x);
        quaternion.setY(transform_odom_to_base_link.transform.rotation.y);
        quaternion.setZ(transform_odom_to_base_link.transform.rotation.z);

        // クォータニオンからRPYを取得
        double roll, pitch, yaw;
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom";
        Eigen::Matrix3d rotationMatrix_yaw;
        rotationMatrix_yaw << cos(yaw), - sin(yaw),  0.0,
                              sin(yaw),   cos(yaw),  0.0,
                                   0.0,        0.0,  1.0;
        Eigen::Vector3d originalCheckpoint;
        for(int n = 0; n < checkpoints_position.size(); n++)
        {
            originalCheckpoint << checkpoints_position[n][0],
                                  checkpoints_position[n][1],
                                  checkpoints_position[n][2];
            Eigen::Vector3d rotatedCheckpoint = rotationMatrix_yaw * originalCheckpoint;

            transformStamped.child_frame_id = "checkpoint_" + std::to_string(n);
            transformStamped.transform.translation.x = transform_odom_to_base_link.transform.translation.x + rotatedCheckpoint(0);
            transformStamped.transform.translation.y = transform_odom_to_base_link.transform.translation.y + rotatedCheckpoint(1);
            transformStamped.transform.translation.z = transform_odom_to_base_link.transform.translation.z + rotatedCheckpoint(2);
            transformStamped.transform.rotation.w    = transform_odom_to_base_link.transform.rotation.w;
            transformStamped.transform.rotation.x    = transform_odom_to_base_link.transform.rotation.x;
            transformStamped.transform.rotation.y    = transform_odom_to_base_link.transform.rotation.y;
            transformStamped.transform.rotation.z    = transform_odom_to_base_link.transform.rotation.z;
            static_br.sendTransform(transformStamped);
            setCheckpoints = true;
        }
    }
    catch (tf2::TransformException& ex)
    {
        setCheckpoints = false;
        ROS_WARN("%s", ex.what());
        return;
    }
}

void Setting_start_goal::callback_mode(const std_msgs::String::ConstPtr &msg)
{
    mode_msg = msg->data.c_str();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "set_start_goal");
    Setting_start_goal setting_start_goal;

    ros::spin();
}