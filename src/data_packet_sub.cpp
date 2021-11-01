#include "ros/ros.h"
#include "Blutonomy/data_packet.h"

void dataPacketCallback(const Blutonomy::data_packet::ConstPtr& msg) {
    ROS_INFO("Data Packet #[%f]", msg->packet_number.data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("data_packet_node", 1000, dataPacketCallback);
    ros::Rate rate(10.0);
    ros::spin();

    return 0;
}