#include <vector>
#include <cmath>
#include <iostream> //Maybe delete or use ROSINFO

//Tidy up --------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"
#include "Blutonomy/data_packet.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include<bits/stdc++.h>

#include <chrono>
#include <random>
#include <thread>
#include <mutex>

#include "marker_helper.h"

class Vehicle
{
public:
    Vehicle(ros::NodeHandle nh);
    
    ~Vehicle();
    
    ros::NodeHandle nh_;

    void mainFunction(void);
    void control(void);


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

    visualization_msgs::Marker resultant_vector_marker_;
    visualization_msgs::MarkerArray marker_array_;
    
    ros::Publisher marker_pub_;

    ros::Publisher lateral_thrust_2_;
    ros::Publisher lateral_thrust_angle_2_;

    ros::Publisher left_thrust_2_;
    ros::Publisher left_thrust_angle_2_;

    ros::Publisher right_thrust_2_;
    ros::Publisher right_thrust_angle_2_;

    std_msgs::Float32 lat_thrust_2_;
    std_msgs::Float32 l_thrust_2_;
    std_msgs::Float32 r_thrust_2_;

    std_msgs::Float32 lat_thrust_angle_2_;
    std_msgs::Float32 l_thrust_angle_2_;
    std_msgs::Float32 r_thrust_angle_2_;

    geometry_msgs::Point object_of_interest_;
    geometry_msgs::Point A_to_object_;
    const int speed_of_sound_ = 1500;
    const int lat_to_meters = 111139;
    const int long_to_meters = 111190;
    std::vector<float> data_packet_;
    int packet_number_;

    ros::Publisher acknowledgement_;
    ros::Publisher data_packet_pub_;
    std_msgs::Int64 acknowledgement_data_;
    ros::Subscriber vehicle_A_GPS_sub_;
    ros::Subscriber vehicle_B_GPS_sub_;
    ros::Subscriber data_packet_sub_;
    std::vector<double> range_circles;
    std::vector<float> net_vector;
    std::vector<double> net_vector_mag;
    std::vector<std::vector<float>> movement_vectors;
    std::vector<double> vehicle_A_GPS_;
    std::vector<double> vehicle_B_GPS_;
    std::vector<std::vector<double>> vehicle_A_GPS_history_;
    std::vector<std::vector<double>> vehicle_B_GPS_history_;
    std::vector<std::vector<std::vector<float>>> solutions;
    double difference;
    std::vector<double> resultant_;
    std::vector<double> start_position_b;
    bool localised_;
    std::mutex mutex_;

    double rangeCalc(double time_sent);
    double simulateRange(void);
    void dataPacketCallback(const Blutonomy::data_packet::ConstPtr& msg);
    void vehicleAGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg);
    void vehicleBGPSCallback(const sensor_msgs::NavSatFixConstPtr &msg);
    void acknowledgement(void);
    void localisation(void);
    void publishDataPacket(void);
    std::vector<float> explorationVehicleVector(void);
    std::vector<std::vector<float>> vectorLocalisation(std::vector<float> net_vector, double d1, double d2);
    void purePursuit(double centreDistance, double range);
};