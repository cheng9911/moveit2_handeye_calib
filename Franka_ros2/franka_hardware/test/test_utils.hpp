// Copyright (c) 2023 Franka Robotics GmbH
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

#include <gmock/gmock.h>
#include <exception>
#include <fstream>
#include <rclcpp/rclcpp.hpp>

#include <franka_hardware/franka_hardware_interface.hpp>
#include <franka_hardware/model.hpp>
#include <franka_hardware/robot.hpp>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

const std::string k_position_controller{"position"};
const std::string k_velocity_controller{"velocity"};
const std::string k_effort_controller{"effort"};
const std::string k_joint_name{"joint"};
const std::string k_arm_id{"fr3"};
const size_t k_number_of_joints{7};
const double k_EPS{1e-5};

class MockModel : public franka_hardware::Model {};

class MockRobot : public franka_hardware::Robot {
 public:
  MOCK_METHOD(void, initializeJointPositionInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianVelocityInterface, (), (override));
  MOCK_METHOD(void, initializeCartesianPoseInterface, (), (override));
  MOCK_METHOD(void, initializeTorqueInterface, (), (override));
  MOCK_METHOD(void, initializeJointVelocityInterface, (), (override));
  MOCK_METHOD(void, stopRobot, (), (override));
  MOCK_METHOD(franka::RobotState, readOnce, (), (override));
  MOCK_METHOD(MockModel*, getModel, (), (override));
  MOCK_METHOD(void, writeOnce, ((const std::vector<double>&)), (override));
  MOCK_METHOD(void,
              writeOnce,
              ((const std::vector<double>&), (const std::vector<double>&)),
              (override));
  MOCK_METHOD(void,
              setJointStiffness,
              (const franka_msgs::srv::SetJointStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setCartesianStiffness,
              (const franka_msgs::srv::SetCartesianStiffness::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, setLoad, (const franka_msgs::srv::SetLoad::Request::SharedPtr&), (override));
  MOCK_METHOD(void,
              setTCPFrame,
              (const franka_msgs::srv::SetTCPFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setStiffnessFrame,
              (const franka_msgs::srv::SetStiffnessFrame::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setForceTorqueCollisionBehavior,
              (const franka_msgs::srv::SetForceTorqueCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void,
              setFullCollisionBehavior,
              (const franka_msgs::srv::SetFullCollisionBehavior::Request::SharedPtr&),
              (override));
  MOCK_METHOD(void, automaticErrorRecovery, (), (override));
};

/**
 * Gets the path to a directory the given file is in
 * @param path A file path
 * @return std::string The string ending at the last '/'
 */
inline auto getDirectoryName(const std::string& path) -> std::string {
  const size_t last_slash_idx = path.rfind('/');
  if (std::string::npos != last_slash_idx) {
    return path.substr(0, last_slash_idx + 1);
  }
  return "";
}

/**
 * Reads a file into a string
 * @param filename The name of the file to read
 * @return std::string The contents of the file
 */
inline auto readFileToString(const std::string& filename) -> std::string {
  auto file = std::ifstream(filename);
  if (!file.is_open()) {
    std::cerr << "Error opening file: " << filename << std::endl;
    return "";
  }

  auto oss = std::ostringstream();
  oss << file.rdbuf();
  file.close();

  return oss.str();
}

#define TEST_CASE_DIRECTORY getDirectoryName(__FILE__)
