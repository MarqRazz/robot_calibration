#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"

#include <robot_calibration/util/action_server.hpp>

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("robot_calibration_server");
	rclcpp::Logger logger = node->get_logger();

	// Get calibration poses YAML path
	std::string data_source("calibration_poses.yaml");
  if (argc > 1)
  {
    data_source = argv[1];
  }

	CalibratePoseServer calibrate_pose_action_server = CalibratePoseServer(node, data_source);

  rclcpp::shutdown();

	return 0;
}