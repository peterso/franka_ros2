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

#include <franka_example_controllers/move_to_start_multiverse_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MoveToStartMultiverseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
MoveToStartMultiverseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type MoveToStartMultiverseController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  if (num_joints != 7) {
    const std::string error_log = "There are " + std::to_string(num_joints) + " joints.";
    RCLCPP_INFO(get_node()->get_logger(), error_log.c_str());
    return controller_interface::return_type::OK;
  }
  if (send_data_vec.size() != 7) {
    const std::string error_log = "There are " + std::to_string(send_data_vec.size()) + " data.";
    RCLCPP_INFO(get_node()->get_logger(), error_log.c_str());
    return controller_interface::return_type::OK;
  }

  updateJointStates();
  auto trajectory_time = this->get_node()->now() - start_time_;
  auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);
  Vector7d q_desired = motion_generator_output.first;
  bool finished = motion_generator_output.second;
  if (not finished) {
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
    Vector7d tau_d_calculated =
        k_gains_.cwiseProduct(q_desired - q_) + d_gains_.cwiseProduct(-dq_filtered_);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(tau_d_calculated(i));
    }
  } else {
    for (auto& command_interface : command_interfaces_) {
      command_interface.set_value(0);
    }
    this->get_node()->set_parameter({"process_finished", true});
  }

  for (int i = 0; i < num_joints; i++)
  {
    joint_commands[i][0] = q_(i);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn MoveToStartMultiverseController::on_init() {
  q_goal_ << 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4;
  try {
    auto_declare<bool>("process_finished", false);
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  meta_data["world_name"] = "world";
  meta_data["simulation_name"] = "real_panda";
  meta_data["length_unit"] = "m";
  meta_data["angle_unit"] = "rad";
  meta_data["mass_unit"] = "kg";
  meta_data["time_unit"] = "s";
  meta_data["handedness"] = "rhs";

  rclcpp::Parameter server_ip_parameter;
  if (get_node()->get_parameter("server_ip", server_ip_parameter))
  {
    host = "tcp://" + server_ip_parameter.as_string();
  }
  else
  {
    host = "tcp://192.168.0.104";
  }

  server_port = "7000";
  client_port = "1111";

  for (int i = 0; i < 7; i++) {    
    const std::string actuator_name = "panda_joint" + std::to_string(i+1);
    send_objects[actuator_name] = {"joint_rvalue"};
    actuators[actuator_name] = i;
    
    init_joint_commands[i] = q_goal_[i];
    joint_commands[i] = (double*)calloc(3, sizeof(double));
    joint_commands[i][0] = init_joint_commands[i];
  }

  const std::string log_info = "Connecting to " + host + ":" + server_port + "...";
  RCLCPP_INFO(get_node()->get_logger(), log_info.c_str());

  connect();

  RCLCPP_INFO(get_node()->get_logger(), "Connected.");

  *world_time = 0.0;

  sim_start_time = get_time_now();

  commnunicate_thread = std::thread([this]() {
    while (rclcpp::ok()) {
      communicate();
    }
    disconnect();
  });

  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveToStartMultiverseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = get_node()->get_parameter("arm_id").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                 num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                 num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MoveToStartMultiverseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);
  start_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
}

void MoveToStartMultiverseController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool MoveToStartMultiverseController::init_objects(bool) {
  return send_objects.size() > 0;
}

void MoveToStartMultiverseController::start_connect_to_server_thread() {
  connect_to_server();
}

void MoveToStartMultiverseController::wait_for_connect_to_server_thread_finish() {}

void MoveToStartMultiverseController::start_meta_data_thread() {
  send_and_receive_meta_data();
}

void MoveToStartMultiverseController::wait_for_meta_data_thread_finish() {}

void MoveToStartMultiverseController::bind_request_meta_data() {
  // Create JSON object and populate it
  request_meta_data_json.clear();
  request_meta_data_json["meta_data"]["world_name"] = meta_data["world_name"];
  request_meta_data_json["meta_data"]["simulation_name"] = meta_data["simulation_name"];
  request_meta_data_json["meta_data"]["length_unit"] = meta_data["length_unit"];
  request_meta_data_json["meta_data"]["angle_unit"] = meta_data["angle_unit"];
  request_meta_data_json["meta_data"]["mass_unit"] = meta_data["mass_unit"];
  request_meta_data_json["meta_data"]["time_unit"] = meta_data["time_unit"];
  request_meta_data_json["meta_data"]["handedness"] = meta_data["handedness"];

  for (const std::pair<const std::string, std::set<std::string>>& send_object : send_objects) {
    for (const std::string& attribute_name : send_object.second) {
      request_meta_data_json["send"][send_object.first].append(attribute_name);
    }
  }

  request_meta_data_str = request_meta_data_json.toStyledString();
  RCLCPP_INFO(get_node()->get_logger(), request_meta_data_str.c_str());
}

void MoveToStartMultiverseController::bind_response_meta_data() {
  RCLCPP_INFO(get_node()->get_logger(), response_meta_data_str.c_str());
}

void MoveToStartMultiverseController::bind_api_callbacks() {}

void MoveToStartMultiverseController::bind_api_callbacks_response() {}

void MoveToStartMultiverseController::init_send_and_receive_data() {
  for (const std::pair<const std::string, std::set<std::string>>& send_object :
       send_objects) {
    const int i = actuators[send_object.first];
    for (const std::string& attribute_name : send_object.second) {
      if (strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
          strcmp(attribute_name.c_str(), "joint_rvalue") == 0) {
        send_data_vec.emplace_back(&joint_commands.at(i)[0]);
      } else if (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
                 strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0) {
        send_data_vec.emplace_back(&joint_commands.at(i)[0]);
      }
    }
  }
}

void MoveToStartMultiverseController::bind_send_data() {
  *world_time = get_time_now() - sim_start_time;
  for (size_t i = 0; i < send_buffer.buffer_double.size; i++) {
    send_buffer.buffer_double.data[i] = *send_data_vec[i];
  }
}

void MoveToStartMultiverseController::bind_receive_data() {

}

void MoveToStartMultiverseController::clean_up() {
  send_data_vec.clear();
}

void MoveToStartMultiverseController::reset() {
  sim_start_time = get_time_now();
  for (int i = 0; i < num_joints; i++) {
    joint_commands[i][0] = init_joint_commands[i];
    joint_commands[i][1] = 0.0;
    joint_commands[i][2] = 0.0;
  }
}
}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MoveToStartMultiverseController,
                       controller_interface::ControllerInterface)
