#include "marker_helper.h"

visualization_msgs::Marker MarkerHelper::generateCircle(double radius, geometry_msgs::Point centre, int id)
{
    // set marker data
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    // create cirle
    int points = 64;
    for (float angle = 0; angle <= 2*M_PI; angle += 2*M_PI/points)
    {
        geometry_msgs::Point point;
        point.x = radius*cos(angle) + centre.x;
        point.y = radius*sin(angle) + centre.y;
        point.z = 0;
        marker.points.push_back(point);
    }
    return marker;
}