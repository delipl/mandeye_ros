#pragma once
#include <deque>
#include <mandeye_ros/point_types.hpp>
#include <memory>
#include <pcl/point_cloud.h>
#include <string>

namespace mandeye
{
bool saveLaz(const std::string& filename, std::deque<pcl::PointCloud<pcl::LivoxPoint>::Ptr>& buffer);
}