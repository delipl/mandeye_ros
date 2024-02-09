
#include <chrono>
#include <json.hpp>
#include <ostream>
#include <thread>

// #include "gnss.h"
#include "mandeye_ros/save_laz.h"
#include "mandeye_ros/FileSystemClient.h"
#include "mandeye_ros/LivoxClient.h"
#include <chrono>
#include <fstream>
// #include <gpios.h>
#include <iostream>
#include <string>

//configuration for alienware
#define MANDEYE_LIVOX_LISTEN_IP "192.168.1.5"
#define MANDEYE_REPO "/media/usb/"
#define MANDEYE_GPIO_SIM false
#define SERVER_PORT 8003
#define MANDEYE_GNSS_UART "/dev/ttyS0"
namespace mandeye
{
void savePointcloudData(LivoxPointsBufferPtr buffer, const std::string& directory, int chunk)
{
	using namespace std::chrono_literals;
	char lidarName[256];
	snprintf(lidarName, 256, "lidar%04d.laz", chunk);
	std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);
	std::cout << "Savig lidar buffer of size " << buffer->size() << " to " << lidarFilePath << std::endl;
	saveLaz(lidarFilePath.string(), buffer);
	//        std::ofstream lidarStream(lidarFilePath.c_str());
	//        for (const auto &p : *buffer){
	//            lidarStream<<p.point.x << " "<<p.point.y <<"  "<<p.point.z << " "<< p.point.tag << " " << p.timestamp << "\n";
	//        }
	system("sync");
	return;
}

void saveLidarList(const std::unordered_map<uint32_t, std::string>& lidars, const std::string& directory, int chunk)
{
	using namespace std::chrono_literals;
	char lidarName[256];
	snprintf(lidarName, 256, "lidar%04d.sn", chunk);
	std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);
	std::cout << "Savig lidar list of size " << lidars.size() << " to " << lidarFilePath << std::endl;

	std::ofstream lidarStream(lidarFilePath);
	for(const auto& [id, sn] : lidars)
	{
		lidarStream << id << " " << sn << "\n";
	}
	system("sync");
	return;
}

void saveImuData(LivoxIMUBufferPtr buffer, const std::string& directory, int chunk)
{
	using namespace std::chrono_literals;
	char lidarName[256];
	snprintf(lidarName, 256, "imu%04d.csv", chunk);
	std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);
	std::cout << "Savig imu buffer of size " << buffer->size() << " to " << lidarFilePath << std::endl;
	std::ofstream lidarStream(lidarFilePath.c_str());
	std::stringstream ss;

	for(const auto& p : *buffer)
	{
		if(p.timestamp > 0)
		{
			ss << p.timestamp << " " << p.point.gyro_x << " " << p.point.gyro_y << " " << p.point.gyro_z << " " << p.point.acc_x << " "
			   << p.point.acc_y << " " << p.point.acc_z << " " << p.laser_id << "\n";
		}
	}
	lidarStream << ss.rdbuf();

	lidarStream.close();
	system("sync");
	return;
}

void saveGnssData(std::deque<std::string>& buffer, const std::string& directory, int chunk)
{
	// if(buffer.size() == 0 && mandeye::gnssClientPtr)
	// {
	// 	mandeye::gnssClientPtr->startListener(utils::getEnvString("MANDEYE_GNSS_UART", MANDEYE_GNSS_UART), 9600);
	// }
	using namespace std::chrono_literals;
	char lidarName[256];
	snprintf(lidarName, 256, "gnss%04d.gnss", chunk);
	std::filesystem::path lidarFilePath = std::filesystem::path(directory) / std::filesystem::path(lidarName);
	std::cout << "Savig gnss buffer of size " << buffer.size() << " to " << lidarFilePath << std::endl;
	std::ofstream lidarStream(lidarFilePath.c_str());
	std::stringstream ss;

	for(const auto& p : buffer)
	{
		ss << p;
	}
	lidarStream << ss.rdbuf();

	lidarStream.close();
	system("sync");
	return;
}
} // namespace mandeye