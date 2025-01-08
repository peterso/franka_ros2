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

#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "multiverse_client_json.h"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class JointImpedanceMultiverseController : public controller_interface::ControllerInterface,
                                           public MultiverseClientJson {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double elapsed_time_{0.0};
  void updateJointStates();

 private:
  void start_connect_to_server_thread() override;

  void wait_for_connect_to_server_thread_finish() override;

  void start_meta_data_thread() override;

  void wait_for_meta_data_thread_finish() override;

  bool init_objects(bool from_request_meta_data = false) override;

  void bind_request_meta_data() override;

  void bind_response_meta_data() override;

  void bind_api_callbacks() override;

  void bind_api_callbacks_response() override;

  void init_send_and_receive_data() override;

  void bind_send_data() override;

  void bind_receive_data() override;

  void clean_up() override;

  void reset() override;

 private:
  std::map<std::string, std::string> meta_data;

  std::map<std::string, std::set<std::string>> send_objects;

  std::map<std::string, std::set<std::string>> receive_objects;

  std::vector<double *> send_data_vec;

  std::vector<double *> receive_data_vec;

  std::vector<std::string> actuator_names;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
      {"position", {}},
      {"velocity", {}},
      {"acceleration", {}}};

  std::map<std::string, int> actuators;

  std::map<std::string, int> joints;

  std::unordered_map<std::string, std::vector<std::string>> actuator_interfaces = {{"position", {}},
                                                                                   {"velocity", {}},
                                                                                   {"effort", {}}};

  std::map<int, double> init_joint_commands;

  std::map<int, double*> joint_states;

  std::map<int, double*> joint_commands;

  double sim_start_time;

  std::thread commnunicate_thread;
};

}  // namespace franka_example_controllers
