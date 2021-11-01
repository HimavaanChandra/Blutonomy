#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <gazebo_msgs/ModelStates.h>

void modelStatesCallback(const gazebo_msgs::ModelStates &msg)
{
    // find model number for wamv and wamv2
    int wamv;
    int wamv2;

    for (int i = 0; i < msg.name.size(); ++i)
    {
        if (msg.name.at(i) == "wamv") wamv = i;
        else if (msg.name.at(i) == "wamv2") wamv2 = i;
    }

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped wamvTransform;
    geometry_msgs::TransformStamped wamv2Transform;

    // set transform data for wamv
    wamvTransform.header.stamp = ros::Time::now();
    wamvTransform.header.frame_id = "world";
    wamvTransform.child_frame_id = "/wamv/base_link";
    wamvTransform.transform.translation.x = msg.pose.at(wamv).position.x;
    wamvTransform.transform.translation.y = msg.pose.at(wamv).position.y;
    wamvTransform.transform.translation.z = msg.pose.at(wamv).position.z;
    wamvTransform.transform.rotation = msg.pose.at(wamv).orientation;

    // set transform data for wamv2
    wamv2Transform.header.stamp = ros::Time::now();
    wamv2Transform.header.frame_id = "world";
    wamv2Transform.child_frame_id = "/wamv2/base_link";
    wamv2Transform.transform.translation.x = msg.pose.at(wamv2).position.x;
    wamv2Transform.transform.translation.y = msg.pose.at(wamv2).position.y;
    wamv2Transform.transform.translation.z = msg.pose.at(wamv2).position.z;
    wamv2Transform.transform.rotation = msg.pose.at(wamv2).orientation;

    br.sendTransform(wamvTransform);
    br.sendTransform(wamv2Transform);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wamv_tf_publisher");

    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/gazebo/model_states", 0, &modelStatesCallback);

    ros::spin();
    return 0;
};