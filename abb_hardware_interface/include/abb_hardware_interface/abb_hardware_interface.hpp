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
// ros2_control 4.x style: interfaces are created by the framework from the
// URDF; read()/write() move values between the framework storage and the EGM
// wire-side data by interface name.
class ABBSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ABBSystemHardware)

  ROS2_CONTROL_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;

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

  // ros2_control interface names per wire-side joint, in motion_data_
  // traversal order (groups -> units -> joints). Derived once in on_init from
  // the ABB controller description and validated against the URDF joints.
  std::vector<std::string> joint_names_;

  /// Copy all wire-side joint states into the framework state interfaces,
  /// folding the J2-J3 coupling into the copied-out values. The wire-side
  /// states are never mutated, so the transform is idempotent regardless of
  /// whether the wire data is fresh, and a raw (uncoupled) J3 value is never
  /// observable outside this class.
  void applyCouplingAndSetStates();

  // J2-J3 coupling parameters
  bool j23_coupling_ = false;
  double J23_factor = -1.0;

  // Track EGM connection state
  bool egm_connected_ = false;
  // Consecutive read() cycles without a fresh EGM message. The manager's own
  // freshness window is ~20 ms (5 messages at 250 Hz), which UDP jitter and
  // marginal control-loop overruns cross routinely; the connection verdict
  // debounces over kDisconnectDebounceReads cycles so those blips do not
  // fire disconnect/reconnect edges (and the re-seeds hanging off them).
  static constexpr unsigned int kDisconnectDebounceReads = 25;  // ~100 ms at 250 Hz
  unsigned int stale_reads_ = kDisconnectDebounceReads;
  // URDF declares the egm/connected gpio; exported every read cycle.
  bool has_egm_gpio_ = false;
  std::vector<double> last_positions_;
  int unchanged_count_ = 0;
};

}  // namespace abb_hardware_interface
