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

#include "franka_hardware/franka_action_server.hpp"

namespace franka_hardware {

ActionServer::ActionServer(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot)
    : rclcpp::Node("action_server", options), robot_(std::move(robot)) {
  error_recovery_action_server_ = rclcpp_action::create_server<franka_msgs::action::ErrorRecovery>(
      this, "~/error_recovery",
      [](auto /*uuid*/, auto /*goal*/) { return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; },
      [](const auto& /*goal_handle*/) { return rclcpp_action::CancelResponse::ACCEPT; },
      [this](const auto& goal_handle) { errorRecoveryAction(goal_handle); });

  RCLCPP_INFO(get_logger(), "Action server started");
}

auto ActionServer::errorRecoveryAction(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_msgs::action::ErrorRecovery>>&
        goal_handle) -> void {
  auto result = std::make_shared<franka_msgs::action::ErrorRecovery::Result>();
  try {
    robot_->automaticErrorRecovery();
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Automatic recovery succeeded");
  } catch (const franka::CommandException& command_exception) {
    RCLCPP_ERROR(this->get_logger(), "Command exception thrown during automatic error recovery %s",
                 command_exception.what());
    goal_handle->abort(result);
  } catch (const franka::NetworkException& network_exception) {
    RCLCPP_ERROR(this->get_logger(), "Network exception thrown automatic error recovery %s",
                 network_exception.what());
    goal_handle->abort(result);
  }
}

}  // namespace franka_hardware
