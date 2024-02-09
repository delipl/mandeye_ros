#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class LivoxSubscriber : public rclcpp::Node {
public:
    LivoxSubscriber() : Node("livox_subscriber") {
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_topic", 10, std::bind(&LivoxSubscriber::imuCallback, this, std::placeholders::_1));

        pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/pointcloud_topic", 10, std::bind(&LivoxSubscriber::pointcloudCallback, this, std::placeholders::_1));
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Przetwarzanie danych IMU
        // Możesz dodać swój kod tutaj
    }

    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Przetwarzanie chmury punktów
        // Możesz dodać swój kod tutaj
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LivoxSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
