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
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rcutils/logging_macros.h>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rcpputils/split.hpp>
#include <std_msgs/msg/header.hpp>

#include <franka_robot_state_broadcaster/franka_robot_state_broadcaster.hpp>

namespace franka_robot_state_broadcaster {

// Override trylock to customize the locking mechanism
// You are excused for wondering why this is necessary.
// RealtimePublisher::trylock() failure is highly likely due to the 1kHz publish rate.
// Here we force the scheduler to yield our thread, rescheduling in [sleep_time_] microseconds.
// After [try_count_] attempts, we give up. Failure to ever gain Lock results in an error message.
// Hopefully, the next call to update() will be successful.
bool FrankaRobotStateBroadcaster::FrankaRobotStateRealtimePublisher::trylock() {
  int count{0};
  while (++count <= try_count_ &&
         !realtime_tools::RealtimePublisher<franka_msgs::msg::FrankaRobotState>::trylock()) {
    std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_));
  }
  return count <= try_count_;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_init() {
  try {
    param_listener = std::make_shared<ParamListener>(get_node());
    params = param_listener->get_params();

    auto_declare<int>(kLockTryCount, kLock_try_count);
    auto_declare<int>(kLockSleepInterval, kLock_sleep_interval);
    auto_declare<bool>(kLockLogError, kLock_log_error);
    auto_declare<bool>(kLockUpdateSuccess, kLock_update_success);

  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaRobotStateBroadcaster::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = franka_robot_state_->get_state_interface_names();
  return state_interfaces_config;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  params = param_listener->get_params();
  std::string robot_description;
  auto this_node = get_node();
  if (!this_node->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(this_node->get_logger(), "Failed to get robot_description parameter");
    return CallbackReturn::ERROR;
  }
  if (!franka_robot_state_) {
    franka_robot_state_ = std::make_unique<franka_semantic_components::FrankaRobotState>(
        franka_semantic_components::FrankaRobotState(params.arm_id + "/" + state_interface_name,
                                                     robot_description));
  }
  current_pose_stamped_publisher_ = this_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      kCurrentPoseTopic, rclcpp::SystemDefaultsQoS());
  last_desired_pose_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::PoseStamped>(kLastDesiredPoseTopic,
                                                                   rclcpp::SystemDefaultsQoS());
  desired_end_effector_twist_stamped_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::TwistStamped>(kDesiredEETwist,
                                                                    rclcpp::SystemDefaultsQoS());
  measured_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kMeasuredJointStates, rclcpp::SystemDefaultsQoS());
  external_wrench_in_stiffness_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(
          kExternalWrenchInStiffnessFrame, rclcpp::SystemDefaultsQoS());
  external_wrench_in_base_frame_publisher_ =
      this_node->create_publisher<geometry_msgs::msg::WrenchStamped>(kExternalWrenchInBaseFrame,
                                                                     rclcpp::SystemDefaultsQoS());
  external_joint_torques_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kExternalJointTorques, rclcpp::SystemDefaultsQoS());
  desired_joint_states_publisher_ = this_node->create_publisher<sensor_msgs::msg::JointState>(
      kDesiredJointStates, rclcpp::SystemDefaultsQoS());
  try {
    franka_state_publisher = this_node->create_publisher<franka_msgs::msg::FrankaRobotState>(
        "~/" + state_interface_name, rclcpp::SystemDefaultsQoS());

    lock_log_error_ = this_node->get_parameter(kLockLogError).as_bool();
    lock_update_success_ = this_node->get_parameter(kLockUpdateSuccess).as_bool();

    int try_count = this_node->get_parameter(kLockTryCount).as_int();
    int sleep_time = this_node->get_parameter(kLockSleepInterval).as_int();

    if (try_count < 0 || sleep_time < 0) {
      RCLCPP_ERROR(this_node->get_logger(),
                   "lock_try_count AND lock_sleep_interval must be greater than 0");
      return CallbackReturn::ERROR;
    }
    realtime_franka_state_publisher =
        std::make_shared<FrankaRobotStateBroadcaster::FrankaRobotStateRealtimePublisher>(
            franka_state_publisher, try_count, sleep_time);
    franka_robot_state_->initialize_robot_state_msg(realtime_franka_state_publisher->msg_);
  } catch (const std::exception& e) {
    fprintf(stderr,
            "Exception thrown during publisher creation at configure stage with message : %s \n",
            e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_DEBUG(get_node()->get_logger(), "configure successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->assign_loaned_state_interfaces(state_interfaces_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn FrankaRobotStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_robot_state_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaRobotStateBroadcaster::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  if (!realtime_franka_state_publisher->trylock()) {
    if (lock_log_error_) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Failed to lock the realtime publisher after %d attempts",
                   realtime_franka_state_publisher->try_count());
    }

    return lock_update_success_ ? controller_interface::return_type::OK
                                : controller_interface::return_type::ERROR;
  }

  realtime_franka_state_publisher->msg_.header.stamp = time;

  if (!franka_robot_state_->get_values_as_message(realtime_franka_state_publisher->msg_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Failed to get franka state via franka state interface.");
    realtime_franka_state_publisher->unlock();
    return controller_interface::return_type::ERROR;
  }

  realtime_franka_state_publisher->unlockAndPublish();

  const auto& franka_state_msg = realtime_franka_state_publisher->msg_;

  current_pose_stamped_publisher_->publish(franka_state_msg.o_t_ee);

  last_desired_pose_stamped_publisher_->publish(franka_state_msg.o_t_ee_d);

  desired_end_effector_twist_stamped_publisher_->publish(franka_state_msg.o_dp_ee_d);

  external_wrench_in_base_frame_publisher_->publish(franka_state_msg.o_f_ext_hat_k);

  external_wrench_in_stiffness_frame_publisher_->publish(franka_state_msg.k_f_ext_hat_k);

  measured_joint_states_publisher_->publish(franka_state_msg.measured_joint_state);

  external_joint_torques_publisher_->publish(franka_state_msg.tau_ext_hat_filtered);

  desired_joint_states_publisher_->publish(franka_state_msg.desired_joint_state);

  return controller_interface::return_type::OK;
}

}  // namespace franka_robot_state_broadcaster

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_robot_state_broadcaster::FrankaRobotStateBroadcaster,
                       controller_interface::ControllerInterface)
