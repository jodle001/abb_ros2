// Copyright 2020 ROS2-Control Development Team
// Modifications Copyright 2022 PickNik Inc
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

#include <abb_egm_rws_managers/egm_manager.h>
#include <abb_egm_rws_managers/rws_manager.h>
#include <abb_hardware_interface/visibility_control.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace abb_hardware_interface
{
class ABBSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemHardware)

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  ROS2_CONTROL_DRIVER_PUBLIC
  return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // EGM
  abb::robot::RobotControllerDescription robot_controller_description_;
  std::unique_ptr<abb::robot::EGMManager> egm_manager_;

  // Store the state and commands for the robot(s)
  std::vector<double> urcl_ft_sensor_measurements_;
  abb::robot::MotionData motion_data_;

  // Exported ros2_control buffers, decoupled from the EGM wire-side data in
  // motion_data_. Controllers and non-RT readers (e.g. a trajectory
  // controller's goal callback sampling hold positions for a partial goal)
  // only ever see these buffers. The J2-J3 coupling transforms happen in the
  // copy between the two, so a raw (uncoupled) J3 value is never observable
  // outside this class; transforming motion_data_ in place would open a
  // window a concurrent reader can catch mid-transform.
  std::vector<double> exported_state_positions_;
  std::vector<double> exported_state_velocities_;
  std::vector<double> exported_cmd_positions_;
  std::vector<double> exported_cmd_velocities_;

  /// Fold the J2-J3 coupling into the wire-side joint states (which must be
  /// fresh, raw EGM feedback) and copy all joint states into the exported
  /// buffers.
  void applyCouplingAndExportStates();

  // J2-J3 coupling parameters
  bool j23_coupling_ = false;
  double J23_factor = -1.0;

  // Track EGM connection state
  bool egm_connected_ = false;
  std::vector<double> last_positions_;
  int unchanged_count_ = 0;
};

}  // namespace abb_hardware_interface
