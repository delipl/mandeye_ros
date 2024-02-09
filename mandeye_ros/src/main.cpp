#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <filesystem>
#include <fstream>
#include <queue>
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

		timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&LivoxSubscriber::timerCallback, this));
	}

private:
	void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		sensor_msgs::msg::Imu::SharedPtr copied(msg);
		imu_queue.push(copied);
	}

	void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		sensor_msgs::msg::PointCloud2::SharedPtr copied(msg);
		pc2_queue.push(copied);
	}

	void timerCallback()
	{
		if(not pc2_queue.size())
		{
			return;
		}
		RCLCPP_INFO(get_logger(), "Saving...");

		// while(pc2_queue.size())
		// {
		// 	sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg = pc2_queue.front();
		// 	pc2_queue.pop();

		// 	// IMU has higher frequency what means has more messages in the queue
		// 	while(imu_queue.size())
		// 	{
		// 		sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_queue.front();
		// 		imu_queue.pop();
		// 	}
		// }

		saveImuData(imu_queue, "./data", chunk_counter);
		chunk_counter++;
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
	std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pc2_queue;
	std::size_t chunk_counter;

	void saveImuData(std::queue<sensor_msgs::msg::Imu::SharedPtr>& imu_queue, const std::string& directory, int chunk)
	{
		using namespace std::chrono_literals;
		const std::string file_name = "imu" + std::to_string(chunk) + ".csv";
		// std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(file_name);
		const std::string lidarFilePath = directory + "/" + file_name;
		const std::size_t size_to_save = imu_queue.size();
		RCLCPP_INFO_STREAM(get_logger(), "Saving imu buffer of size " << size_to_save << " to " << lidarFilePath);

		std::ofstream lidarStream(lidarFilePath);
		std::stringstream ss;

		for(auto i = 0u; i < size_to_save; ++i)
		{
			sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_queue.front();
			imu_queue.pop();
			ss << static_cast<float>(imu_msg->header.stamp.sec) + imu_msg->header.stamp.nanosec * 1e-9 << " " << imu_msg->angular_velocity.x << " "
			   << imu_msg->angular_velocity.y << " " << imu_msg->angular_velocity.z << " " << imu_msg->linear_acceleration.x << " "
			   << imu_msg->linear_acceleration.y << " " << imu_msg->linear_acceleration.z << std::endl;
		}

		lidarStream << ss.rdbuf();
		lidarStream.close();
		system("sync");
		return;
	}
};

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<LivoxSubscriber>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
