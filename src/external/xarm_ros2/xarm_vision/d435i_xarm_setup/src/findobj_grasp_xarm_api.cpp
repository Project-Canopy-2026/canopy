/* Copyright 2025 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 * Author: Vinman <vinman@ufactory.cc>
 ============================================================================*/
#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <std_msgs/msg/bool.hpp>
#include <xarm_api/xarm_ros_client.h>
#include <vector>

// Caution: motion services from "xarm_api" did not verify the trajectory (collision and singularity check) in advance，
// Thus motion may fail during execution.

static const std::string target_frame = "link_base";
static const std::string source_frame = "object_1";

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("findobj_grasp_xarm_api", node_options);

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string hw_ns;
    node->get_parameter_or("hw_ns", hw_ns, std::string("xarm"));

    RCLCPP_INFO(node->get_logger(), "robot_type: %s, hw_ns: %s, dof: %d", robot_type.c_str(), hw_ns.c_str(), dof);

    // should run this node under "xarm" namespace !
    // *********** configuration:
    xarm_api::XArmROSClient xarm_c;
    xarm_c.init(node, hw_ns);

    int ret;

    xarm_c.motion_enable(true);
    
    std::vector<float> gripper_tcp_offset = {0, 0, 172, 0, 0, 0};
    if (robot_type == "lite") {
        gripper_tcp_offset[2] = 61.1;  // lite vacuum gripper
    }
    xarm_c.set_tcp_offset(gripper_tcp_offset);

    xarm_c.set_mode(0);
    xarm_c.set_state(0);

    if (robot_type == "lite") {
        ret = xarm_c.set_vacuum_gripper(false);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Vacuum Gripper off failed");
            exit(-1);
        }
    }
    else {
        // init xarm gripper
        xarm_c.set_gripper_enable(true);
        xarm_c.set_gripper_speed(3000);
        // open gripper
        ret = xarm_c.set_gripper_position(850, true);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Gripper open failed");
            exit(-1);
        }
    }

    // here, give the start pose:
    std::vector<float> start_pose = {350, 0, 200, M_PI, 0, 0};
    if (robot_type == "lite") {
        start_pose[0] = 250;
    } 
    ret = xarm_c.set_position(start_pose, -1, 160, 1000, 0, true);
    if (ret) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Second, wait for the recognition result from find_object_3d:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::shared_ptr<tf2_ros::TransformListener> listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    geometry_msgs::msg::TransformStamped transform;

    std::string errMsg;
    
    if (!tf_buffer->canTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(15.0), &errMsg))
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "Unable to get pose from TF : " << errMsg.c_str());
      exit(-1);
    } else {
      try {
        // transform = tf_buffer->lookupTransform(
        //   target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(
        //     0.5));
        transform = tf_buffer->lookupTransform(target_frame, source_frame, tf2::TimePoint());
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(
          node->get_logger(),
          "Error in lookupTransform of " << source_frame << " in " << target_frame << " : " << e.what());
        exit(-1);
      }
    }
    
    double x = transform.transform.translation.x * 1000.0;
    double y = transform.transform.translation.y * 1000.0;
    double z = transform.transform.translation.z * 1000.0;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;

    tf2::Matrix3x3 mat(tf2::Quaternion{qx, qy, qz, qw});
    tf2Scalar yaw, pitch, roll;
    mat.getEulerYPR(yaw, pitch, roll);

    RCLCPP_WARN(node->get_logger(), "listener: x: %lf, y: %lf, z: %lf, q: [%lf, %lf, %lf, %lf], rpy=[%lf, %lf, %lf]", x, y, z, qw, qx, qy, qz, roll, pitch, yaw);
    rclcpp::sleep_for(std::chrono::seconds(1));

    // *********** Third, go to 40mm above target:
    std::vector<float> target_pose = {(float)x, (float)y, (float)z + 40, M_PI, 0, (float)yaw};
    ret = xarm_c.set_position(target_pose, -1, 160, 1000, 0, true);
    if (ret) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Fourth, go to -10mm below recognized target, for grasp:
    target_pose[2] = z - 10;
    ret = xarm_c.set_position(target_pose, -1, 160, 1000, 0, true);
    if (ret) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Fifth step, grasp:
    rclcpp::sleep_for(std::chrono::seconds(1));
    if (robot_type == "lite") {
        ret = xarm_c.set_vacuum_gripper(true, true);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Vacuum Gripper on failed");
            exit(-1);
        }
    }
    else {
        ret = xarm_c.set_gripper_position(120, true);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Gripper close failed");
            exit(-1);
        }
    }

    target_pose[2] = start_pose[2];
    ret = xarm_c.set_position(target_pose, -1, 160, 1000, 0, true);
    if (ret) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Last step, go back to start pose:
    rclcpp::sleep_for(std::chrono::seconds(1));
    ret = xarm_c.set_position(start_pose, -1, 160, 1000, 0, true);
    if (ret) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    return 0;

}

