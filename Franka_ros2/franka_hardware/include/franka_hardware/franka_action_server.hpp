// Copyright (c) 2024 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>

#include "franka/exception.h"
#include "franka_hardware/robot.hpp"

#include <franka_msgs/action/error_recovery.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace franka_hardware {

/**
 * Node implementing the action server for franka hardware related capabilities.
 */
class ActionServer : public rclcpp::Node {
 public:
  /**
   * @brief Construct the franka ActionServer
   *
   * @param options rclcpp::NodeOptions Options for the ROS 2 node
   * @param robot std::shared_ptr<Robot> The robot backend
   */
  ActionServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot);

 private:
  std::shared_ptr<Robot> robot_;
  rclcpp_action::Server<franka_msgs::action::ErrorRecovery>::SharedPtr
      error_recovery_action_server_;

  /**
   * @brief Callback function for the error recovery action server
   *
   * @param goal_handle The goal handle for the error recovery action
   */
  auto errorRecoveryAction(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>>&
          goal_handle) -> void;
};

}  // namespace franka_hardware
