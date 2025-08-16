// Copyright (c) 2025 Franka Robotics GmbH
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

#include <rclcpp/rclcpp.hpp>

#include <franka/logging/logging_sink_interface.hpp>

namespace franka_hardware {

/**
 * Implements the logger interface of libfranka to use the ROS logger.
 */
class RosLibfrankaLogger : public franka::LoggingSinkInterface {
 public:
  explicit RosLibfrankaLogger(const rclcpp::Logger& logger) : logger_(logger) {}
  ~RosLibfrankaLogger() override = default;

  // Inherited via LoggingSinkInterface
  [[nodiscard]] auto getName() const -> std::string override { return "RosLibfrankaLogger"; }
  auto logInfo(const std::string& message) -> void override;
  auto logWarn(const std::string& message) -> void override;
  auto logError(const std::string& message) -> void override;

 private:
  rclcpp::Logger logger_;
};

}  // namespace franka_hardware
