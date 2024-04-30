#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <mandeye_ros/point_types.hpp>
#include <mandeye_ros/save_laz.h>
#include <pcl_conversions/pcl_conversions.h>
class LivoxSubscriber : public rclcpp::Node
{
public:
	LivoxSubscriber()
		: Node("mandeye")
	{

		cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

		rclcpp::SubscriptionOptions options;
		options.callback_group = cb_group_;
		imu_subscription_ =
			this->create_subscription<sensor_msgs::msg::Imu>("/livox/imu", 10, std::bind(&LivoxSubscriber::imuCallback, this, std::placeholders::_1), options);

		pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/livox/lidar", 10, std::bind(&LivoxSubscriber::pointcloudCallback, this, std::placeholders::_1), options	);

		timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&LivoxSubscriber::timerCallback, this), cb_group_);
	}

private:
	void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
	{
		sensor_msgs::msg::Imu::SharedPtr copied(msg);
		m1.lock();
		imu_deque.push_back(copied);
		m1.unlock();
	}

	void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
	{
		std::deque<mandeye::Point> temp;
		sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");
		sensor_msgs::PointCloud2ConstIterator<float> i_it(*msg, "intensity");

		int point_counter = 0;
		for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it, ++i_it) {
			mandeye::Point point;
			point.point.x() = *x_it;
			point.point.y() = *y_it;
			point.point.z() = *z_it;
			point.intensity = *i_it;

			point.timestamp = rclcpp::Time(msg->header.stamp).nanoseconds();
			temp.push_back(point);
			point_counter++;
		}

		m2.lock();
		pc2_deque.insert(pc2_deque.end(), temp.begin(), temp.end());
		m2.unlock();
	}

	void timerCallback()
	{
		if(not pc2_deque.size() or not imu_deque.size())
		{
			return;
		}
		RCLCPP_INFO(get_logger(), "Saving...");


		std::scoped_lock(m1, m2);
		std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque_copy{imu_deque};
		std::deque<mandeye::Point> pc2_deque_copy{pc2_deque};
		imu_deque.clear();
		pc2_deque.clear();

		saveImuData(imu_deque_copy, "./data", chunk_counter);
		savePointcloudData(pc2_deque_copy, "./data", chunk_counter);

		chunk_counter++;
	}

	std::mutex m1;
	std::mutex m2;
	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
	std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_deque;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
	std::deque<mandeye::Point> pc2_deque;
	std::size_t chunk_counter;

	void saveImuData(std::deque<sensor_msgs::msg::Imu::SharedPtr>& imu_deque, const std::string& directory, int chunk)
	{
		using namespace std::chrono_literals;
		std::stringstream file_name_ss;
		file_name_ss << "imu" << std::setfill('0') << std::setw(4) << std::to_string(chunk) << ".csv";

		// std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(file_name);
		const std::string lidarFilePath = directory + "/" + file_name_ss.str();
		const std::size_t size_to_save = imu_deque.size();
		RCLCPP_INFO_STREAM(get_logger(), "Saving imu buffer of size " << size_to_save << " to " << lidarFilePath);

		std::ofstream lidarStream(lidarFilePath);
		std::stringstream ss;

		for(auto i = 0u; i < size_to_save; ++i)
		{
			sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_deque.front();

			imu_deque.pop_front();
			ss << rclcpp::Time(imu_msg->header.stamp).nanoseconds() << " " <<  imu_msg->angular_velocity.x << " " << imu_msg->angular_velocity.y
            << " " << imu_msg->angular_velocity.z << " " << imu_msg->linear_acceleration.x << " " << imu_msg->linear_acceleration.y << " "
            << imu_msg->linear_acceleration.z;
		}

		lidarStream << ss.rdbuf();
		lidarStream.close();
		system("sync");
		return;
	}

	void savePointcloudData(std::deque<mandeye::Point>& pc2_deque, const std::string& directory, int chunk)
	{
		using namespace std::chrono_literals;
		std::stringstream file_name_ss;
		file_name_ss << "lidar" << std::setfill('0') << std::setw(4) << std::to_string(chunk) << ".laz";

		const std::string lidarFilePath = directory + "/" + file_name_ss.str();
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
