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

#include <franka_example_controllers/joint_impedance_multiverse_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointImpedanceMultiverseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceMultiverseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceMultiverseController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {

  if (num_joints != 7) {
    const std::string error_log = "There are " + std::to_string(num_joints) + " joints.";
    RCLCPP_INFO(get_node()->get_logger(), error_log.c_str());
    return controller_interface::return_type::OK;
  }
  if (receive_data_vec.size() != 7) {
    const std::string error_log = "There are " + std::to_string(receive_data_vec.size()) + " data.";
    RCLCPP_INFO(get_node()->get_logger(), error_log.c_str());
    return controller_interface::return_type::OK;
  }

  updateJointStates();

  Vector7d q_goal = initial_q_;
  for (int i = 0; i < num_joints; i++)
  {
    joint_states[i][0] = q_(i);
    joint_states[i][1] = dq_(i);
    q_goal(i) = joint_commands[i][0];
  }

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * dq_;
  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_goal - q_) + d_gains_.cwiseProduct(-dq_filtered_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_[i].set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceMultiverseController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "");
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

  std::vector<double> init_joint_values = {0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4};
  for (int i = 0; i < 7; i++) {
    const std::string joint_name = "joint" + std::to_string(i+1);
    send_objects[joint_name] = {"joint_rvalue", "joint_angular_velocity"};
    joints[joint_name] = i;
    joint_states[i] = (double*)calloc(2, sizeof(double));
    
    const std::string actuator_name = "actuator" + std::to_string(i+1);
    receive_objects[actuator_name] = {"cmd_joint_rvalue"};
    actuators[actuator_name] = i;
    
    init_joint_commands[i] = init_joint_values[i];
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

CallbackReturn JointImpedanceMultiverseController::on_configure(
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

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceMultiverseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  return CallbackReturn::SUCCESS;
}

void JointImpedanceMultiverseController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

bool JointImpedanceMultiverseController::init_objects(bool) {
  return send_objects.size() > 0 && receive_objects.size() > 0;
}

void JointImpedanceMultiverseController::start_connect_to_server_thread() {
  connect_to_server();
}

void JointImpedanceMultiverseController::wait_for_connect_to_server_thread_finish() {}

void JointImpedanceMultiverseController::start_meta_data_thread() {
  send_and_receive_meta_data();
}

void JointImpedanceMultiverseController::wait_for_meta_data_thread_finish() {}

void JointImpedanceMultiverseController::bind_request_meta_data() {
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

  for (const std::pair<const std::string, std::set<std::string>>& receive_object :
       receive_objects) {
    for (const std::string& attribute_name : receive_object.second) {
      request_meta_data_json["receive"][receive_object.first].append(attribute_name);
    }
  }

  request_meta_data_str = request_meta_data_json.toStyledString();
  RCLCPP_INFO(get_node()->get_logger(), request_meta_data_str.c_str());
}

void JointImpedanceMultiverseController::bind_response_meta_data() {
  RCLCPP_INFO(get_node()->get_logger(), response_meta_data_str.c_str());
}

void JointImpedanceMultiverseController::bind_api_callbacks() {}

void JointImpedanceMultiverseController::bind_api_callbacks_response() {}

void JointImpedanceMultiverseController::init_send_and_receive_data() {
  for (const std::pair<const std::string, std::set<std::string>>& send_object :
       send_objects) {
    const int i = joints[send_object.first];
    for (const std::string& attribute_name : send_object.second) {
      if (strcmp(attribute_name.c_str(), "joint_tvalue") == 0 ||
          strcmp(attribute_name.c_str(), "joint_rvalue") == 0) {
        send_data_vec.emplace_back(&joint_states.at(i)[0]);
      } else if (strcmp(attribute_name.c_str(), "joint_linear_velocity") == 0 ||
                 strcmp(attribute_name.c_str(), "joint_angular_velocity") == 0) {
        send_data_vec.emplace_back(&joint_states.at(i)[0]);
      }
    }
  }
  for (const std::pair<const std::string, std::set<std::string>>& receive_object :
       receive_objects) {
    const int i = actuators[receive_object.first];
    for (const std::string& attribute_name : receive_object.second) {
      if (strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
          strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0) {
        receive_data_vec.emplace_back(&joint_commands.at(i)[0]);
      } else if (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
                 strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0) {
        receive_data_vec.emplace_back(&joint_commands.at(i)[1]);
      }
    }
  }
}

void JointImpedanceMultiverseController::bind_send_data() {
  *world_time = get_time_now() - sim_start_time;
  for (size_t i = 0; i < send_buffer.buffer_double.size; i++) {
    send_buffer.buffer_double.data[i] = *send_data_vec[i];
  }
}

void JointImpedanceMultiverseController::bind_receive_data() {
  for (size_t i = 0; i < receive_buffer.buffer_double.size; i++) {
    *receive_data_vec[i] = receive_buffer.buffer_double.data[i];
  }
}

void JointImpedanceMultiverseController::clean_up() {
  send_data_vec.clear();
  receive_data_vec.clear();
}

void JointImpedanceMultiverseController::reset() {
  sim_start_time = get_time_now();
  for (int i = 0; i < num_joints; i++) {
    joint_commands[i][0] = initial_q_(i);
    joint_commands[i][1] = 0.0;
    joint_commands[i][2] = 0.0;
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceMultiverseController,
                       controller_interface::ControllerInterface)
