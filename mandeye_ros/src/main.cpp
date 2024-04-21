#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>

#include <mandeye_ros/point_types.hpp>
#include <mandeye_ros/save_laz.h>
#include <pcl_conversions/pcl_conversions.h>
class LivoxSubscriber : public rclcpp::Node
{
public:
	LivoxSubscriber()
		: Node("mandeye")
	{
		imu_subscription_ =
			this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu", 10, std::bind(&LivoxSubscriber::imuCallback, this, std::placeholders::_1));

		pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/livox/lidar", 10, std::bind(&LivoxSubscriber::pointcloudCallback, this, std::placeholders::_1));

		timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LivoxSubscriber::timerCallback, this));
	}

private:
	void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		sensor_msgs::msg::Imu::SharedPtr copied(msg);
		imu_deque.push_back(copied);
	}

	void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		pcl::PCLPointCloud2 temp_cloud;
		pcl::PointCloud<pcl::LivoxPoint>::Ptr cloud(new pcl::PointCloud<pcl::LivoxPoint>);
		pcl_conversions::toPCL(*msg, temp_cloud);
		pcl::fromPCLPointCloud2(temp_cloud, *cloud);
		pc2_deque.push_back(cloud);
	}

	void timerCallback()
	{
		if(not pc2_deque.size() or not imu_deque.size())
		{
			return;
		}
		RCLCPP_INFO(get_logger(), "Saving...");

		saveImuData(imu_deque, "./data", chunk_counter);
		savePointcloudData(pc2_deque, "./data", chunk_counter);

		chunk_counter++;
	}

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
	std::deque<pcl::PointCloud<pcl::LivoxPoint>::Ptr> pc2_deque;
	std::size_t chunk_counter;

	void saveImuData(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_deque, const std::string& directory, int chunk)
	{
		using namespace std::chrono_literals;
		const std::string file_name = "imu" + std::to_string(chunk) + ".csv";
		// std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(file_name);
		const std::string lidarFilePath = directory + "/" + file_name;
		const std::size_t size_to_save = imu_deque.size();
		RCLCPP_INFO_STREAM(get_logger(), "Saving imu buffer of size " << size_to_save << " to " << lidarFilePath);

		std::ofstream lidarStream(lidarFilePath);
		std::stringstream ss;

		for(auto i = 0u; i < size_to_save; ++i)
		{
			sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_deque.front();
			auto timestamp = static_cast<std::size_t>(imu_msg->header.stamp.sec * 1e9 + imu_msg->header.stamp.nanosec);

			imu_deque.pop_front();
			ss << std::fixed << std::setw(20) << std::setprecision(14) << timestamp << " " << std::fixed << std::setw(20)
			   << imu_msg->angular_velocity.x << " " << std::fixed << std::setw(20) << imu_msg->angular_velocity.y << " " << std::fixed
			   << std::setw(20) << imu_msg->angular_velocity.z << " " << std::fixed << std::setw(20) << imu_msg->linear_acceleration.x << " "
			   << std::fixed << std::setw(20) << imu_msg->linear_acceleration.y << " " << std::fixed << std::setw(20)
			   << imu_msg->linear_acceleration.z << std::endl;
		}

		lidarStream << ss.rdbuf();
		lidarStream.close();
		system("sync");
		return;
	}

	void savePointcloudData(std::deque<pcl::PointCloud<pcl::LivoxPoint>::Ptr>& pc2_deque, const std::string& directory, int chunk)
	{
		using namespace std::chrono_literals;
		const std::string file_name = "lidar" + std::to_string(chunk) + ".laz";
		const std::string lidarFilePath = directory + "/" + file_name;
		const std::size_t size_to_save = pc2_deque.size();
		// std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);

		RCLCPP_INFO_STREAM(get_logger(), "Saving lidar buffer of size " << size_to_save << " to " << lidarFilePath);
		if(not mandeye::saveLaz(lidarFilePath, pc2_deque))
		{
			RCLCPP_ERROR_STREAM(get_logger(), "Failed to save " << chunk_counter << " chunk!");
		}

		system("sync");
		return;
	}
};

// void savePointcloudData(LivoxPointsBufferPtr buffer, const std::string& directory, int chunk)
// {
// 	using namespace std::chrono_literals;
// 	char lidarName[256];
// 	snprintf(lidarName, 256, "lidar%04d.laz", chunk);
// 	std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);
// 	std::cout << "Savig lidar buffer of size " << buffer->size() << " to " << lidarFilePath << std::endl;
// 	saveLaz(lidarFilePath.string(), buffer);
// 	//        std::ofstream lidarStream(lidarFilePath.c_str());
// 	//        for (const auto &p : *buffer){
// 	//            lidarStream<<p.point.x << " "<<p.point.y <<"  "<<p.point.z << " "<< p.point.tag << " " << p.timestamp << "\n";
// 	//        }
// 	system("sync");
// 	return;
// }

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<LivoxSubscriber>();
	executor.add_node(node);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
