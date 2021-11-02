#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <iostream>
#include <mutex>
#include <visualization_msgs/Marker.h>

std::mutex mtx;
gazebo_msgs::ModelStates _modelStates;

void modelStatesCallback(const gazebo_msgs::ModelStates &msg)
{
    std::unique_lock<std::mutex> lck (mtx);
    _modelStates = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wamv_tf_publisher");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/gazebo/model_states", 0, &modelStatesCallback);
    ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/visualization_marker",1);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped wamvTransform;
    geometry_msgs::TransformStamped wamv2Transform;

    ros::Rate loop_rate(60);
    while (ros::ok && _modelStates.name.size() < 1)
    {
        // wait for data
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("got data");

    int count = 0;
    while (ros::ok)
    {
        gazebo_msgs::ModelStates modelStates;
        {
            std::unique_lock<std::mutex> lck (mtx);
            modelStates = _modelStates;
        }

        // find model number for wamv and wamv2
        int wamv;
        int wamv2;

        for (int i = 0; i < modelStates.name.size(); ++i)
        {
            if (modelStates.name.at(i) == "wamv")
                wamv = i;
            else if (modelStates.name.at(i) == "wamv2")
                wamv2 = i;
        }

        // set transform data for wamv
        wamvTransform.header.stamp = ros::Time::now();
        wamvTransform.header.frame_id = "world";
        wamvTransform.child_frame_id = "/wamv/base_link";
        wamvTransform.transform.translation.x = modelStates.pose.at(wamv).position.x;
        wamvTransform.transform.translation.y = modelStates.pose.at(wamv).position.y;
        wamvTransform.transform.translation.z = modelStates.pose.at(wamv).position.z;
        wamvTransform.transform.rotation = modelStates.pose.at(wamv).orientation;

        // set transform data for wamv2
        wamv2Transform.header.stamp = ros::Time::now();
        wamv2Transform.header.frame_id = "world";
        wamv2Transform.child_frame_id = "/wamv2/base_link";
        wamv2Transform.transform.translation.x = modelStates.pose.at(wamv2).position.x;
        wamv2Transform.transform.translation.y = modelStates.pose.at(wamv2).position.y;
        wamv2Transform.transform.translation.z = modelStates.pose.at(wamv2).position.z;
        wamv2Transform.transform.rotation = modelStates.pose.at(wamv2).orientation;

        br.sendTransform(wamvTransform);
        br.sendTransform(wamv2Transform);

        // publish wamv markers
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time();
        marker.id = count+100;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position = modelStates.pose.at(wamv).position;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker_pub.publish(marker);

        count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
};