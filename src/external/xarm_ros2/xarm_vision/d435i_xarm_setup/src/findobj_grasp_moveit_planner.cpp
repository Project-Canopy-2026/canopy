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
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>
#include <xarm_msgs/srv/set_float32.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/gripper_move.hpp>
#include <xarm_msgs/srv/vacuum_gripper_ctrl.hpp>
#include <stdlib.h>
#include <vector>

#define SERVICE_CALL_FAILED 999

static const std::string target_frame = "link_base";
static const std::string source_frame = "object_1";

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request(rclcpp::Node::SharedPtr& node, std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return false;
    }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, success=%d", client->get_service_name(), res->success);
    return res->success;
}

template<typename ServiceT, typename SharedRequest = typename ServiceT::Request::SharedPtr>
int call_request2(rclcpp::Node::SharedPtr& node, std::shared_ptr<ServiceT> client, SharedRequest req)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result_future) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to call service %s", client->get_service_name());
        return SERVICE_CALL_FAILED;
    }
    auto res = result_future.get();
    RCLCPP_INFO(node->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}

// static rclcpp::Node::SharedPtr node;
// static rclcpp::Client<xarm_msgs::srv::PlanSingleStraight>::SharedPtr line_client;
// static rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_client;
static std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> line_plan_req = std::make_shared<xarm_msgs::srv::PlanSingleStraight::Request>();
static std::shared_ptr<xarm_msgs::srv::PlanExec::Request> exec_req = std::make_shared<xarm_msgs::srv::PlanExec::Request>();

// static rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr gripper_enable_client;
// static rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr gripper_spd_client;
// static rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_client;
static std::shared_ptr<xarm_msgs::srv::SetInt16::Request> gripper_enable_req = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
static std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> gripper_spd_req = std::make_shared<xarm_msgs::srv::SetFloat32::Request>();
static std::shared_ptr<xarm_msgs::srv::GripperMove::Request> gripper_move_req = std::make_shared<xarm_msgs::srv::GripperMove::Request>();
static std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> vacuum_gripper_ctrl_req = std::make_shared<xarm_msgs::srv::VacuumGripperCtrl::Request>();

int set_gripper_enable(rclcpp::Node::SharedPtr& node, rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr& client, std::shared_ptr<xarm_msgs::srv::SetInt16::Request> &req)
{
    req->data = 1;
    return call_request2(node, client, req);
}

int set_gripper_speed(rclcpp::Node::SharedPtr& node, rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr& client, std::shared_ptr<xarm_msgs::srv::SetFloat32::Request> &req, double speed)
{
    req->data = speed;
    return call_request2(node, client, req);
}

int set_gripper_position(rclcpp::Node::SharedPtr& node, rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr& client, std::shared_ptr<xarm_msgs::srv::GripperMove::Request> &req, double pos)
{
    req->pos = pos;
    return call_request2(node, client, req);
}

int set_vacuum_gripper(rclcpp::Node::SharedPtr& node, rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr& client, std::shared_ptr<xarm_msgs::srv::VacuumGripperCtrl::Request> &req, bool on, bool wait=false, int hardware_version=1)
{
    req->on = on;
    req->wait = wait;
    req->hardware_version = hardware_version;
    return call_request2(node, client, req);
}

int move(rclcpp::Node::SharedPtr& node, rclcpp::Client<xarm_msgs::srv::PlanSingleStraight>::SharedPtr& line_client, std::shared_ptr<xarm_msgs::srv::PlanSingleStraight::Request> &line_plan_req, rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr& exec_client, std::shared_ptr<xarm_msgs::srv::PlanExec::Request> &exec_req, geometry_msgs::msg::Pose target)
{
    line_plan_req->target = target;
    if (call_request(node, line_client, line_plan_req))
    {
        exec_req->wait = true;
        return call_request(node, exec_client, exec_req);
    }
    return false;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grasp_test_xarm_planner", node_options);
    RCLCPP_INFO(node->get_logger(), "grasp_test_xarm_planner start");

    int dof;
    node->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node->get_parameter_or("robot_type", robot_type, std::string("xarm"));
    std::string hw_ns;
    node->get_parameter_or("hw_ns", hw_ns, std::string("xarm"));

    RCLCPP_INFO(node->get_logger(), "robot_type: %s, hw_ns: %s, dof: %d", robot_type.c_str(), hw_ns.c_str(), dof);

    rclcpp::Client<xarm_msgs::srv::PlanSingleStraight>::SharedPtr line_client = node->create_client<xarm_msgs::srv::PlanSingleStraight>("xarm_straight_plan");
    rclcpp::Client<xarm_msgs::srv::PlanExec>::SharedPtr exec_client = node->create_client<xarm_msgs::srv::PlanExec>("xarm_exec_plan");

    rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr gripper_enable_client;
    rclcpp::Client<xarm_msgs::srv::SetFloat32>::SharedPtr gripper_spd_client;
    rclcpp::Client<xarm_msgs::srv::GripperMove>::SharedPtr gripper_move_client;
    rclcpp::Client<xarm_msgs::srv::VacuumGripperCtrl>::SharedPtr vacuum_gripper_ctrl_client;

    int ret;
    if (robot_type == "lite") {
        vacuum_gripper_ctrl_client = node->create_client<xarm_msgs::srv::VacuumGripperCtrl>("/" + hw_ns + "/set_vacuum_gripper");
        ret = set_vacuum_gripper(node, vacuum_gripper_ctrl_client, vacuum_gripper_ctrl_req, false);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Vacuum Gripper off failed");
            exit(-1);
        }
    }
    else {
        gripper_enable_client = node->create_client<xarm_msgs::srv::SetInt16>("/" + hw_ns + "/set_gripper_enable");
        gripper_spd_client = node->create_client<xarm_msgs::srv::SetFloat32>("/" + hw_ns + "/set_gripper_speed");
        gripper_move_client = node->create_client<xarm_msgs::srv::GripperMove>("/" + hw_ns + "/set_gripper_position");

        set_gripper_enable(node, gripper_enable_client, gripper_enable_req);
        set_gripper_speed(node, gripper_spd_client, gripper_spd_req, 3000.0);
        ret = set_gripper_position(node, gripper_move_client, gripper_move_req, 850.0);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Gripper open failed");
            exit(-1);
        }
    }
    

    // here, give the start pose:
    geometry_msgs::msg::Pose start_pose;
    start_pose.position.x = robot_type == "lite" ? 0.25 : 0.35;
    start_pose.position.y = 0;
    start_pose.position.z = 0.2;

    start_pose.orientation.x = 1;
    start_pose.orientation.y = 0;
    start_pose.orientation.z = 0;
    start_pose.orientation.w = 0;

    if (!move(node, line_client, line_plan_req, exec_client, exec_req, start_pose)) {
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

    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    double z = transform.transform.translation.z;

    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    RCLCPP_WARN(node->get_logger(), "recognition listener: x: %lf, y: %lf, z: %lf, q: [%lf, %lf, %lf, %lf]", x, y, z, qw, qx, qy, qz);

    // *********** Third, go to 40mm above target:
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z + 0.04; 
    // target_pose.position.z = start_pose.position.z;

    target_pose.orientation.x = 1;
    target_pose.orientation.y = 0;
    target_pose.orientation.z = 0;
    target_pose.orientation.w = 0;

    if (!move(node, line_client, line_plan_req, exec_client, exec_req, target_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    // *********** Fourth, go to -10mm below recognized target, for grasp:
    target_pose.position.z = z - 0.01;
    if (!move(node, line_client, line_plan_req, exec_client, exec_req, target_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Fifth step, grasp:
    if (robot_type == "lite") {
        ret = set_vacuum_gripper(node, vacuum_gripper_ctrl_client, vacuum_gripper_ctrl_req, true, true);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Vacuum Gripper on failed");
            exit(-1);
        }
    }
    else {
        ret = set_gripper_position(node, gripper_move_client, gripper_move_req, 120.0);
        if (ret) {
            RCLCPP_ERROR(node->get_logger(), "Gripper close failed");
            exit(-1);
        }
    }

    rclcpp::sleep_for(std::chrono::seconds(1));

    target_pose.position.z = start_pose.position.z;
    if (!move(node, line_client, line_plan_req, exec_client, exec_req, target_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    // *********** Last step, go back to start pose:
    if (!move(node, line_client, line_plan_req, exec_client, exec_req, start_pose)) {
        RCLCPP_ERROR(node->get_logger(), "Move failed");
        exit(-1);
    }

    return 0;

}

