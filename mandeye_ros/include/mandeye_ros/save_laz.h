#pragma once
#include <deque>
#include <mandeye_ros/point_types.hpp>
#include <memory>
#include <pcl/point_cloud.h>
#include <string>

namespace mandeye
{
struct Point {
    double timestamp;
    float intensity;
    Eigen::Vector3d point;
};

bool saveLaz(const std::string& filename, std::deque<Point>& buffer);
}