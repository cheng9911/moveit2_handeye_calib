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

#include "ign_ros2_control/model_kdl.h"

#include <algorithm>
#include <array>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/solveri.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>

// Implementation copied from <kdl/isolveri.hpp> because
// KDL::ChainDynSolver inherits *privately* from SolverI ... -.-'
std::string ModelKDL::strError(const int error)
{
  // clang-format off
  switch (error) {
    case KDL::SolverI::E_NOERROR:                 return "No error"; break;
    case KDL::SolverI::E_NO_CONVERGE:             return "Failed to converge"; break;
    case KDL::SolverI::E_UNDEFINED:               return "Undefined value"; break;
    case KDL::SolverI::E_DEGRADED:                return "Converged but degraded solution"; break;
    default: return "UNKNOWN ERROR";
  }
  // clang-format on
}

ModelKDL::ModelKDL(const urdf::Model & model, const std::string & root, const std::string & tip)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
    throw std::invalid_argument("Cannot construct KDL tree from URDF");
  }

  if (!tree.getChain(root, tip, this->chain_)) {
    throw std::invalid_argument(
            "Cannot find chain within URDF tree from root '" + root +
            "' to tip '" + tip + "'. Do these links exist?");
  }
}

std::array<double, 7> ModelKDL::gravity(
  const std::array<double, 7> & q,
  const std::array<double, 3> & gravity_earth) const
{
  KDL::JntArray joint_states, gravity_torques(7);
  KDL::Vector gravity(gravity_earth[0], gravity_earth[1], gravity_earth[2]);
  joint_states.data = Eigen::Matrix<double, 7, 1>(q.data());

  KDL::Chain chain = this->chain_;  // copy

  KDL::ChainDynParam solver(chain, gravity);

  int error = solver.JntToGravity(joint_states, gravity_torques);
  if (error != KDL::SolverI::E_NOERROR) {
    throw std::logic_error("KDL gravity calculation failed with error: " + strError(error));
  }

  std::array<double, 7> result;
  Eigen::VectorXd::Map(&result[0], gravity_torques.data.size()) = gravity_torques.data;

  return result;
}
