#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace MarkerHelper
{
visualization_msgs::Marker generateCircle(double radius, geometry_msgs::Point centre, int id);
visualization_msgs::Marker generateArrow(geometry_msgs::Point start, geometry_msgs::Point end, int id, std::vector<double> color);
}