#include "ros/ros.h"
#include "vehicle.h"
#include "communication.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blutonomy");
    ros::NodeHandle nh;

    std::shared_ptr<Vehicle> vehicle(new Vehicle(nh));
    // std::thread t() -ADD program to run thread
    // std::thread t(&Vehicle::functionName, vehicle); //example

    ros::spin();

    ros::shutdown();
    // t.join()

    return 0;
}