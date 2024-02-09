#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <realtime_tools/realtime_box.h>

class LivoxSubscriber : public rclcpp::Node
{
public:
	LivoxSubscriber()
		: Node("livox_subscriber")
	{
		imu_subscription_ =
			this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu", 10, std::bind(&LivoxSubscriber::imuCallback, this, std::placeholders::_1));

		pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/livox/lidar", 10, std::bind(&LivoxSubscriber::pointcloudCallback, this, std::placeholders::_1));

		timer_ = this->create_wall_timer(std::chrono::nanoseconds(1000), std::bind(&LivoxSubscriber::timerCallback, this));
	}

private:
	void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
        sensor_msgs::msg::Imu::SharedPtr copied(msg);
		imu_box.set(copied);
	}

	void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
        sensor_msgs::msg::PointCloud2::SharedPtr copied(msg);
		pc2_box.set(copied);
	}

	void timerCallback()
	{
		sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg;
		sensor_msgs::msg::Imu::SharedPtr imu_msg;
		pc2_box.get(std::ref(pc2_msg));
		imu_box.get(std::ref(imu_msg));
		if(pc2_msg)
		{
			RCLCPP_INFO(get_logger(), "Got PointCloud2 message!");
		}
		if(imu_msg)
		{
			RCLCPP_INFO(get_logger(), "Got Imu message!");
		}
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	realtime_tools::RealtimeBox<sensor_msgs::msg::Imu::SharedPtr> imu_box;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
	realtime_tools::RealtimeBox<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_box;
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<LivoxSubscriber>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
