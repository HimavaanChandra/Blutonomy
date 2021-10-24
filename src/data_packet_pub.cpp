#include "ros/ros.h"
#include "data_packet_msg_cpp/data_packet.h"

#include <ctime>
#include <chrono>
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_packet_node");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<data_packet_msg_cpp::data_packet>("data_packet_node", 100);
    
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        data_packet_msg_cpp::data_packet msg;

        // Example values 
        msg.packet_number.data = 1;
        msg.x.data = 4;
        msg.y.data = 3;
        msg.z.data = 2;
        msg.timestamp.data = 1634532260266; //UNIX ???
        msg.speed_of_sound.data = 1500;
        msg.depth.data = 10;
        msg.distance_moved.data = 5;
        msg.direction_moved.data = 15;

        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}