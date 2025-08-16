#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <hardware_interface/component_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include "test_utils.hpp"

using namespace std::chrono_literals;

class FrankaActionServerTests
    : public ::testing::TestWithParam<
          std::pair<std::function<void(std::shared_ptr<MockRobot> mock_robot)>,
                    rclcpp_action::ResultCode>> {
 public:
  auto SetUp() -> void override {
    auto urdf_string = readFileToString(TEST_CASE_DIRECTORY + arm_id + ".urdf");
    auto parsed_hardware_infos = hardware_interface::parse_control_resources_from_urdf(urdf_string);
    auto number_of_expected_hardware_components = 1;

    ASSERT_EQ(parsed_hardware_infos.size(), number_of_expected_hardware_components);

    default_hardware_info = parsed_hardware_infos[0];
    default_franka_hardware_interface.on_init(default_hardware_info);
  }

 protected:
  std::string arm_id{"fr3"};
  std::shared_ptr<MockRobot> default_mock_robot = std::make_shared<MockRobot>();
  hardware_interface::HardwareInfo default_hardware_info;
  franka_hardware::FrankaHardwareInterface default_franka_hardware_interface{default_mock_robot,
                                                                             arm_id};
  /* Helper function to get the response of a action service */
  template <typename action_client_type>
  void get_action_service_response(
      std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
      const std::string& action_name,
      rclcpp_action::ResultCode result_code);
};

template <typename action_client_type>
void FrankaActionServerTests::get_action_service_response(
    std::function<void(std::shared_ptr<MockRobot> mock_robot)> mock_function,
    const std::string& action_name,
    rclcpp_action::ResultCode result_code) {
  mock_function(default_mock_robot);

  auto node = rclcpp::Node::make_shared("test_node");

  auto client = rclcpp_action::create_client<action_client_type>(node, action_name);
  if (!client->wait_for_action_server(20s)) {
    ASSERT_TRUE(false) << "Action not available after waiting";
  }
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bool is_finished = false;
  auto goal_msg = typename action_client_type::Goal();
  auto send_goal_options = typename rclcpp_action::Client<action_client_type>::SendGoalOptions();
  send_goal_options.goal_response_callback = [&](const auto& future_result) {
    auto goal_handle = future_result.get();
    ASSERT_TRUE(goal_handle);
  };
  send_goal_options.feedback_callback = [&](auto, auto) { ASSERT_TRUE(false); };
  send_goal_options.result_callback = [&](const auto& result) {
    ASSERT_EQ(result.code, result_code);
    is_finished = true;
  };

  auto action_accepted = client->async_send_goal(goal_msg);
  auto start_point = std::chrono::system_clock::now();
  auto end_point = start_point + 5s;
  while (action_accepted.wait_for(0s) != std::future_status::ready) {
    executor.spin_some();

    ASSERT_LE(std::chrono::system_clock::now(), end_point);
  }
  auto goal_handle = action_accepted.get();

  auto result = client->async_get_result(goal_handle, send_goal_options.result_callback);
  start_point = std::chrono::system_clock::now();
  end_point = start_point + 5s;
  while (!is_finished || result.wait_for(0s) != std::future_status::ready) {
    executor.spin_some();

    ASSERT_LE(std::chrono::system_clock::now(), end_point);
  }

  ASSERT_TRUE(is_finished);
}

TEST_P(FrankaActionServerTests,
       whenErrorRecoveryActionTriggered_thenErrorRecoveryServiceCallExecuted) {
  auto param = GetParam();

  get_action_service_response<franka_msgs::action::ErrorRecovery>(
      param.first, "action_server/error_recovery", param.second);
}

INSTANTIATE_TEST_SUITE_P(
    FrankaActionServerTestsInstantiation,
    FrankaActionServerTests,
    ::testing::Values(std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery()).Times(1);
                          },
                          rclcpp_action::ResultCode::SUCCEEDED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::CommandException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED),
                      std::make_pair(
                          [](std::shared_ptr<MockRobot> mock_robot) {
                            EXPECT_CALL(*mock_robot, automaticErrorRecovery())
                                .Times(1)
                                .WillRepeatedly(testing::Throw(franka::NetworkException("")));
                          },
                          rclcpp_action::ResultCode::ABORTED)));
