#include <vector>
#include <cmath>
#include <iostream> //Maybe delete or use ROSINFO

//Tidy up --------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"

#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <random>
#include <thread>

class Vehicle
{
public:
    Vehicle(ros::NodeHandle nh);
    ~Vehicle();
    void mainFunction(void); //Delete
    ros::NodeHandle nh_;

private:
    //tidy up-----------------------
    cv::Mat image_;

    //control
    ros::Publisher lateral_thrust_;
    ros::Publisher lateral_thrust_angle_;

    ros::Publisher left_thrust_;
    ros::Publisher left_thrust_angle_;

    ros::Publisher right_thrust_;
    ros::Publisher right_thrust_angle_;

    std_msgs::Float32 lat_thrust_;
    std_msgs::Float32 l_thrust_;
    std_msgs::Float32 r_thrust_;

    std_msgs::Float32 lat_thrust_angle_;
    std_msgs::Float32 l_thrust_angle_;
    std_msgs::Float32 r_thrust_angle_;

    double speed_of_sound_; //maybe change over time
    std::vector<float> data_packet_;

    ros::Publisher acknowledgement_;
    std_msgs::Int64 acknowledgement_data_;
    ros::Subscriber vehicle_A_GPS_sub_;
    ros::Subscriber vehicle_B_GPS_sub_;
    ros::Subscriber data_packet_sub_;
    std::vector<double> range_circles;
    std::vector<double> net_vector;
    std::vector<double> net_vector_mag;
    std::vector<std::vector<double>> movement_vectors;
    std::vector<double> vehicle_A_GPS_;
    std::vector<double> vehicle_B_GPS_;
    std::vector<std::vector<double>> vehicle_A_GPS_history_;
    std::vector<std::vector<double>> vehicle_B_GPS_history_;
    std::vector<std::vector<std::vector<double>>> solutions;
    std::vector<std::vector<double>> difference;

    void control(void);
    double rangeCalc(void);
    void dataPacketCallback(void); // change to "this" PMS style
    void vehicleAGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg);
    void vehicleBGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg);
    void acknowledgement(void);
    void localisation(void);
    std::vector<double> explorationVehicleVector(void);
    std::vector<std::vector<double>> vectorLocalisation(std::vector<double> net_vector, double d1, double d2);

    // image_transport::ImageTransport it_;
    // image_transport::Publisher image_pub_; /*!< image publisher*/

    //use this if splitting the robots in the class instead.
    // struct Robot
    // {
    //     double x_;                     /*!< robot's x coordinate*/
    //     double y_;                     /*!< robot's y coordinate*/
    //     double angle_;                 /*!< robot's angle */
    //     bool obstacle_;                /*!< boolean if there is an object 0.2m in front of the robot*/
    //     geometry_msgs::Twist control_; /*!< Controller of the robot*/
    //     std::vector<float> ranges_;    /*!< Laser sensor data*/
    //     std::mutex mtx_;               /*!< mutex to lock data*/
    // };
};