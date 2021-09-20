#include <vector>
#include <cmath>
#include <iostream> //Maybe delete or use ROSINFO

//Tidy up --------------------------------------------------------
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf/transform_datatypes.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class Communication
{
public:
    Communication(ros::NodeHandle nh);
    ~Communication();
    ros::NodeHandle nh_;

private:
};