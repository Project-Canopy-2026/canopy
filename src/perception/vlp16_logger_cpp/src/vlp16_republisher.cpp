#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/qos.hpp"

class VLP16Republisher : public rclcpp::Node
{
public:
    VLP16Republisher()
    : Node("vlp16_republisher")
    {
        // QoS for high-frequency LiDAR
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "vlp16/depth_pcd", qos,
            std::bind(&VLP16Republisher::callback, this, std::placeholders::_1));

        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "vlp16/depth_pcd_repub", qos);

        RCLCPP_INFO(this->get_logger(), "VLP16 Republisher started");
    }

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // republish point cloud data
        pub_->publish(*msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VLP16Republisher>());
    rclcpp::shutdown();
    return 0;
}
