#pragma once

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <std_msgs/msg/string.hpp>
#include <robot_calibration_msgs/msg/calibration_data.hpp>
#include <robot_calibration_msgs/msg/capture_config.hpp>
#include <robot_calibration_msgs/action/calibrate_pose.hpp>

#include <robot_calibration/util/capture_manager.hpp>

class CalibratePoseServer
{
  public:
  CalibratePoseServer(const rclcpp::Node::SharedPtr& node, std::string robot_poses_path);

  private:
  using CalibratePose = robot_calibration_msgs::action::CalibratePose;
  using GoalHandleCalibratePose = rclcpp_action::ServerGoalHandle<CalibratePose>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<CalibratePose>::SharedPtr action_server_;

  std::string robot_poses_path_;

  std_msgs::msg::String description_msg; // will contain URDF
  std::vector<robot_calibration_msgs::msg::CalibrationData> data;

  robot_calibration::CaptureManager capture_manager;
  std::vector<std::string> calibration_steps;

  // Load a set of calibration poses from YAML or bag file
  std::vector<robot_calibration_msgs::msg::CaptureConfig> robot_poses;
  void load_robot_poses();

  // Action server callbacks
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CalibratePose::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCalibratePose> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleCalibratePose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleCalibratePose> goal_handle);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};
