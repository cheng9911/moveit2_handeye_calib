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

#include "franka_hardware/ros_libfranka_logger.hpp"

namespace franka_hardware {

void RosLibfrankaLogger::logInfo(const std::string& message) {
  RCLCPP_INFO(logger_, "%s", message.c_str());
}

void RosLibfrankaLogger::logWarn(const std::string& message) {
  RCLCPP_WARN(logger_, "%s", message.c_str());
}

void RosLibfrankaLogger::logError(const std::string& message) {
  RCLCPP_ERROR(logger_, "%s", message.c_str());
}

}  // namespace franka_hardware
