#include "ros/ros.h"
#include "vehicle.h"
#include "communication.h"
#include <thread>   

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blutonomy");
    ros::NodeHandle nh;

    std::shared_ptr<Vehicle> vehicle(new Vehicle(nh));
    std::thread t(&Vehicle::control, vehicle);
    std::thread t2(&Vehicle::mainFunction, vehicle);

    ros::spin();

    ros::shutdown();
    t.join();
    t2.join();

    return 0;
}