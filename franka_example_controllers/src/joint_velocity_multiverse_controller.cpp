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

#include <franka_example_controllers/default_robot_behavior_utils.hpp>
#include <franka_example_controllers/joint_velocity_multiverse_controller.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "franka/rate_limiting.h"

using namespace std::chrono_literals;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointVelocityMultiverseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityMultiverseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::return_type JointVelocityMultiverseController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
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
  // const double max_vel = 1.5;
  for (int i = 0; i < num_joints; i++) {
    if (num_joints == 7) {
      // double last_commanded_acceleration = state_interfaces_.at(21 + i).get_value();
      // double cmd_joint_vel = franka::limitRate(1.5, -1.5, 5, 3000, cmd_joint_vel, *receive_data_vec[i], last_commanded_acceleration);
      double cmd_joint_vel = *receive_data_vec[i];
      const std::string log_info = std::to_string(i) + ": " + std::to_string(cmd_joint_vel);
      RCLCPP_INFO(get_node()->get_logger(), log_info.c_str());
      command_interfaces_[i].set_value(cmd_joint_vel);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointVelocityMultiverseController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
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

  const std::string log_info = "Connect to " + host + ":" + server_port;
  RCLCPP_INFO(get_node()->get_logger(), log_info.c_str());

  for (int i = 1; i < 8; i++) {
    joint_names.push_back("actuator" + std::to_string(i));
  }
  for (const std::string& joint_name : joint_names) {
    receive_objects[joint_name] = {"cmd_joint_angular_velocity"};

    joint_states[joint_name] = (double*)calloc(2, sizeof(double));
    joint_commands[joint_name] = (double*)calloc(3, sizeof(double));
    init_joint_positions[joint_name] = 0.0;
  }

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

CallbackReturn JointVelocityMultiverseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo = get_node()->get_parameter("gazebo").as_bool();

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

  if (!is_gazebo) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

    auto future_result = client->async_send_request(request);
    future_result.wait_for(1000ms);

    auto success = future_result.get();
    if (!success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityMultiverseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

// std::vector<hardware_interface::StateInterface>
// JointVelocityMultiverseController::export_state_interfaces() {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   for (const std::string& joint_name : joint_interfaces["position"]) {
//     state_interfaces.emplace_back(joint_name, "position", &joint_states[joint_name][0]);
//   }

//   for (const std::string& joint_name : joint_interfaces["velocity"]) {
//     state_interfaces.emplace_back(joint_name, "velocity", &joint_states[joint_name][1]);
//   }

//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface>
// JointVelocityMultiverseController::export_command_interfaces() {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

//   for (const std::string& joint_name : actuator_interfaces["position"]) {
//     command_interfaces.emplace_back(joint_name, "position", &joint_commands[joint_name][0]);
//   }

//   for (const std::string& joint_name : actuator_interfaces["velocity"]) {
//     command_interfaces.emplace_back(joint_name, "velocity", &joint_commands[joint_name][1]);
//   }

//   for (const std::string& joint_name : actuator_interfaces["effort"]) {
//     command_interfaces.emplace_back(joint_name, "effort", &joint_commands[joint_name][2]);
//   }

//   return command_interfaces;
// }

// JointVelocityMultiverseController::~JointVelocityMultiverseController() {
//   commnunicate_thread.join();
// }

bool JointVelocityMultiverseController::init_objects(bool) {
  return receive_objects.size() > 0;
}

void JointVelocityMultiverseController::start_connect_to_server_thread() {
  connect_to_server();
}

void JointVelocityMultiverseController::wait_for_connect_to_server_thread_finish() {}

void JointVelocityMultiverseController::start_meta_data_thread() {
  send_and_receive_meta_data();
}

void JointVelocityMultiverseController::wait_for_meta_data_thread_finish() {}

void JointVelocityMultiverseController::bind_request_meta_data() {
  // Create JSON object and populate it
  request_meta_data_json.clear();
  request_meta_data_json["meta_data"]["world_name"] = meta_data["world_name"];
  request_meta_data_json["meta_data"]["simulation_name"] = meta_data["simulation_name"];
  request_meta_data_json["meta_data"]["length_unit"] = meta_data["length_unit"];
  request_meta_data_json["meta_data"]["angle_unit"] = meta_data["angle_unit"];
  request_meta_data_json["meta_data"]["mass_unit"] = meta_data["mass_unit"];
  request_meta_data_json["meta_data"]["time_unit"] = meta_data["time_unit"];
  request_meta_data_json["meta_data"]["handedness"] = meta_data["handedness"];

  // for (const std::pair<const std::string, std::set<std::string>>& send_object : send_objects) {
  //   for (const std::string& attribute_name : send_object.second) {
  //     request_meta_data_json["send"][send_object.first].append(attribute_name);
  //   }
  // }

  for (const std::pair<const std::string, std::set<std::string>>& receive_object :
       receive_objects) {
    for (const std::string& attribute_name : receive_object.second) {
      request_meta_data_json["receive"][receive_object.first].append(attribute_name);
    }
  }

  request_meta_data_str = request_meta_data_json.toStyledString();
}

void JointVelocityMultiverseController::bind_response_meta_data() {}

void JointVelocityMultiverseController::bind_api_callbacks() {}

void JointVelocityMultiverseController::bind_api_callbacks_response() {}

void JointVelocityMultiverseController::init_send_and_receive_data() {
  RCLCPP_INFO(get_node()->get_logger(), std::to_string(receive_objects.size()).c_str());
  for (const std::pair<const std::string, std::set<std::string>>& receive_object :
       receive_objects) {
    for (const std::string& attribute_name : receive_object.second) {
      if (strcmp(attribute_name.c_str(), "cmd_joint_tvalue") == 0 ||
          strcmp(attribute_name.c_str(), "cmd_joint_rvalue") == 0) {
        receive_data_vec.emplace_back(&joint_states.at(receive_object.first)[0]);
      } else if (strcmp(attribute_name.c_str(), "cmd_joint_linear_velocity") == 0 ||
                 strcmp(attribute_name.c_str(), "cmd_joint_angular_velocity") == 0) {
        receive_data_vec.emplace_back(&joint_states.at(receive_object.first)[1]);
      }
    }
  }
}

void JointVelocityMultiverseController::bind_send_data() {
  *world_time = get_time_now() - sim_start_time;
  // for (size_t i = 0; i < send_buffer.buffer_double.size; i++) {
  //   send_buffer.buffer_double.data[i] = *send_data_vec[i];
  // }
}

void JointVelocityMultiverseController::bind_receive_data() {
  for (size_t i = 0; i < receive_buffer.buffer_double.size; i++) {
    *receive_data_vec[i] = receive_buffer.buffer_double.data[i];
  }
}

void JointVelocityMultiverseController::clean_up() {
  // send_data_vec.clear();
  receive_data_vec.clear();
}

void JointVelocityMultiverseController::reset() {
  sim_start_time = get_time_now();
  for (const std::string& joint_name : joint_names) {
    joint_commands[joint_name][0] = init_joint_positions[joint_name];
    joint_commands[joint_name][1] = 0.0;
    joint_commands[joint_name][2] = 0.0;
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityMultiverseController,
                       controller_interface::ControllerInterface)
