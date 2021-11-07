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
    marker.scale.x = 0.5;
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

visualization_msgs::Marker MarkerHelper::generateArrow(geometry_msgs::Point start, geometry_msgs::Point end, int id, std::vector<double> color)
{
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time();
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.points.resize(2);
    arrow.points.at(0) = start;
    arrow.points.at(1) = end;
    arrow.scale.x = 0.8;
    arrow.scale.y = 2.5;
    arrow.scale.z = 4;
    arrow.color.a = 1.0;
    arrow.color.r = color.at(0);
    arrow.color.g = color.at(1);
    arrow.color.b = color.at(2);
    return arrow;
}