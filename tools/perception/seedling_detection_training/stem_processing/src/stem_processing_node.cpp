#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

class PointCloudCropBoxNode : public rclcpp::Node
{
public:
  PointCloudCropBoxNode() : Node("pointcloud_cropbox_node")
  {
    // Declare parameters with defaults
    this->declare_parameter("min_x", -5.0);
    this->declare_parameter("max_x",  5.0);
    this->declare_parameter("min_y", -5.0);
    this->declare_parameter("max_y",  5.0);
    this->declare_parameter("min_z", -2.0);
    this->declare_parameter("max_z",  2.0);
    this->declare_parameter("translation", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("rotation", std::vector<double>{0.0, 0.0, 0.0});

    // Get initial values
    getParams();

    // Watch for parameter changes
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PointCloudCropBoxNode::paramCallback, this, std::placeholders::_1));

    // Sub / pub
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", 10,
      std::bind(&PointCloudCropBoxNode::pointcloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/filtered_pointcloud", 10);

    RCLCPP_INFO(this->get_logger(), "CropBox filter node started.");
  }

private:
  void getParams()
  {
    this->get_parameter("min_x", min_x_);
    this->get_parameter("max_x", max_x_);
    this->get_parameter("min_y", min_y_);
    this->get_parameter("max_y", max_y_);
    this->get_parameter("min_z", min_z_);
    this->get_parameter("max_z", max_z_);
    this->get_parameter("translation", translation_);
    this->get_parameter("rotation", rotation_);
  }

  // Called whenever a parameter changes
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> &params)
  {
    for (const auto &p : params) {
      if (p.get_name() == "min_x") min_x_ = p.as_double();
      else if (p.get_name() == "max_x") max_x_ = p.as_double();
      else if (p.get_name() == "min_y") min_y_ = p.as_double();
      else if (p.get_name() == "max_y") max_y_ = p.as_double();
      else if (p.get_name() == "min_z") min_z_ = p.as_double();
      else if (p.get_name() == "max_z") max_z_ = p.as_double();
      else if (p.get_name() == "translation") translation_ = p.as_double_array();
      else if (p.get_name() == "rotation") rotation_ = p.as_double_array();
    }

    RCLCPP_INFO(this->get_logger(),
      "Updated CropBox params: x[%f,%f], y[%f,%f], z[%f,%f], "
      "trans=(%f,%f,%f), rot=(%f,%f,%f)",
      min_x_, max_x_, min_y_, max_y_, min_z_, max_z_,
      translation_[0], translation_[1], translation_[2],
      rotation_[0], rotation_[1], rotation_[2]);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *cloud);

    pcl::CropBox<pcl::PointXYZRGB> crop;
    crop.setInputCloud(cloud);

    crop.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0));
    crop.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));
    crop.setTranslation(Eigen::Vector3f(
      static_cast<float>(translation_[0]),
      static_cast<float>(translation_[1]),
      static_cast<float>(translation_[2])));
    crop.setRotation(Eigen::Vector3f(
      static_cast<float>(rotation_[0]),
      static_cast<float>(rotation_[1]),
      static_cast<float>(rotation_[2])));

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
    crop.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header = msg->header;
    pub_->publish(output);
  }

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // Parameters
  double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
  std::vector<double> translation_;
  std::vector<double> rotation_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudCropBoxNode>());
  rclcpp::shutdown();
  return 0;
}
