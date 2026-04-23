/* Copyright 2025 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 * Author: Vinman <vinman@ufactory.cc>
 ============================================================================*/
 #include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "object_recognition_msgs/msg/recognized_object_array.hpp"


rclcpp::Logger logger = rclcpp::get_logger("tf_obj_to_base");
std::shared_ptr<rclcpp::Node> node;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

void recognized_object_callback(const object_recognition_msgs::msg::RecognizedObjectArray::SharedPtr msg){
  if(msg->objects.empty())
  	return;

  if(msg->objects.size()>1)
  	RCLCPP_WARN(logger, "Detected Multiple objects, only broadcasting the first one ..");

  tf2::Vector3 vector(msg->objects.at(0).pose.pose.pose.position.x, msg->objects.at(0).pose.pose.pose.position.y, msg->objects.at(0).pose.pose.pose.position.z);
  tf2::Quaternion quaternion(msg->objects.at(0).pose.pose.pose.orientation.x, msg->objects.at(0).pose.pose.pose.orientation.y, msg->objects.at(0).pose.pose.pose.orientation.z, msg->objects.at(0).pose.pose.pose.orientation.w);
  tf2::Transform transform(quaternion, vector);

  geometry_msgs::msg::TransformStamped transformstamped;
  transformstamped.header.frame_id = "/camera_color_optical_frame";
  transformstamped.header.stamp = node->get_clock()->now();
  transformstamped.child_frame_id = "coke_can";
  tf2::toMsg(transform, transformstamped.transform);

  tf_broadcaster->sendTransform(transformstamped);
}

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node = rclcpp::Node::make_shared("tf_obj_to_base", node_options);

  tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  
  rclcpp::Subscription<object_recognition_msgs::msg::RecognizedObjectArray>::SharedPtr sub = node->create_subscription<object_recognition_msgs::msg::RecognizedObjectArray>("/recognized_object_array", 10, recognized_object_callback);
  
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
};