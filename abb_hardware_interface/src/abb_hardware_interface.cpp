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

#include <abb_hardware_interface/abb_hardware_interface.hpp>
#include <abb_hardware_interface/utilities.hpp>
#include <algorithm>
#include <limits>
#include <boost/exception/diagnostic_information.hpp>

using namespace std::chrono_literals;

namespace abb_hardware_interface {
  static constexpr size_t NUM_CONNECTION_TRIES = 100;
  static const rclcpp::Logger LOGGER = rclcpp::get_logger("ABBSystemHardware");

  CallbackReturn ABBSystemHardware::on_init(
      const hardware_interface::HardwareComponentInterfaceParams &params) {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }

    const auto rws_port = stoi(info_.hardware_parameters["rws_port"]);
    const auto rws_ip = info_.hardware_parameters["rws_ip"];
    const auto is_coupled = info_.hardware_parameters["j23_coupling"];
    j23_coupling_ = is_coupled == "true";

    RCLCPP_INFO_STREAM(LOGGER, "J2-J3 coupling: " << (j23_coupling_ ? "on" : "off"));

    if (rws_ip == "None") {
      RCLCPP_FATAL(LOGGER, "RWS IP not specified");
      return CallbackReturn::ERROR;
    }

    // Get robot controller description from RWS. OmniCore controllers serve RWS 2.0 over
    // TLS only; the xacro emits controller_generation so the right transport is selected.
    const auto controller_generation_it = info_.hardware_parameters.find("controller_generation");
    const auto controller_generation =
        controller_generation_it != info_.hardware_parameters.end() ? controller_generation_it->second : "irc5";
    const auto rws_version = abb::robot::rwsVersionFromControllerGeneration(controller_generation);
    RCLCPP_INFO_STREAM(LOGGER, "Controller generation: " << controller_generation << " (RWS "
                                                         << (rws_version == abb::robot::RWSVersion::v2_0 ? "2.0" :
                                                                                                           "1.0")
                                                         << ")");

    auto rws_manager = abb::robot::makeRWSManager(rws_version, rws_ip, rws_port, "Default User", "robotics");
    const auto robot_controller_description_ =
        abb::robot::utilities::establishRWSConnection(*rws_manager, "IRB1200", true);
    RCLCPP_INFO_STREAM(LOGGER, "Robot controller description:\n"
                       << abb::robot::summaryText(robot_controller_description_));

    for (const hardware_interface::ComponentInfo &joint: info_.joints) {
      if (joint.command_interfaces.size() != 2) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                     joint.command_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                     joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                     joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                     joint.state_interfaces.size());
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                     joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(LOGGER, "Joint '%s' have %s state interface as first state interface. '%s' expected.",
                     joint.name.c_str(), joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return CallbackReturn::ERROR;
      }
    }

    urcl_ft_sensor_measurements_.resize(6);

    // Optional egm/connected gpio: when the URDF declares it, the connection
    // flag is exported as a state interface every read cycle so controllers
    // (the force link) can react to disconnects and reconnects.
    has_egm_gpio_ = false;
    for (const auto &gpio: info_.gpios) {
      for (const auto &si: gpio.state_interfaces) {
        if (gpio.name == "egm" && si.name == "connected") {
          has_egm_gpio_ = true;
        }
      }
    }
    RCLCPP_INFO(LOGGER, "egm/connected state interface: %s",
                has_egm_gpio_ ? "exported" : "not in URDF");

    // Configure EGM
    RCLCPP_INFO(LOGGER, "Configuring EGM interface...");

    // Initialize motion data from robot controller description
    try {
      abb::robot::initializeMotionData(motion_data_, robot_controller_description_);
      abb::robot::SystemStateData system_state_data_;
      rws_manager->collectAndUpdateRuntimeData(system_state_data_, motion_data_);

      // Wire-side states in motion_data_ stay RAW for the lifetime of this
      // object; the J2-J3 coupling is folded in only during the copy to the
      // framework state interfaces (see applyCouplingAndSetStates).
      for (uint i = 0; i < motion_data_.groups.size(); i++) {
        for (uint pp = 0; pp < motion_data_.groups[i].units.size(); pp++) {
          for (uint k = 0; k < motion_data_.groups[i].units[pp].joints.size(); k++) {
            motion_data_.groups[i].units[pp].joints[k].state.velocity = 0.0;
            motion_data_.groups[i].units[pp].joints[k].command.position = motion_data_.groups[i].units[pp].joints[k].
                state.position;
          }
        }
      }
      last_positions_ = std::vector<double>(motion_data_.groups[0].units[0].joints.size(),
                                            std::numeric_limits<double>::quiet_NaN());

      // Derive the ros2_control joint name for every wire-side joint (strip
      // the ABB prefix; extax joints get an ext_ prefix) and validate the set
      // against the URDF, so a mismatch fails here instead of as unclaimable
      // interfaces later.
      joint_names_.clear();
      for (const auto &group: motion_data_.groups) {
        for (const auto &unit: group.units) {
          for (const auto &joint: unit.joints) {
            const auto pos = joint.name.find("joint");
            auto joint_name = joint.name.substr(pos);
            if (group.name == "extax") {
              joint_name = "ext_" + joint.name.substr(pos);
            }
            joint_names_.push_back(joint_name);
          }
        }
      }
      for (const auto &name: joint_names_) {
        const bool in_urdf =
            std::any_of(info_.joints.begin(), info_.joints.end(),
                        [&](const hardware_interface::ComponentInfo &j) { return j.name == name; });
        if (!in_urdf) {
          RCLCPP_FATAL(LOGGER, "Joint '%s' reported by the robot controller is not in the ros2_control URDF",
                       name.c_str());
          return CallbackReturn::ERROR;
        }
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize motion data from robot controller description: "
                                      << boost::diagnostic_information(e));
      return CallbackReturn::ERROR;
    } catch (...) {
      RCLCPP_ERROR_STREAM(LOGGER,
                          "Failed to initialize motion data from robot controller description: unknown exception");
      return CallbackReturn::ERROR;
    }

    // Create channel configuration for each mechanical unit group
    std::vector<abb::robot::EGMManager::ChannelConfiguration> channel_configurations;
    for (const auto &group: robot_controller_description_.mechanical_units_groups()) {
      try {
        const auto egm_port = stoi(info_.hardware_parameters[group.name() + "egm_port"]);
        const auto channel_configuration =
            abb::robot::EGMManager::ChannelConfiguration{static_cast<uint16_t>(egm_port), group};
        channel_configurations.emplace_back(channel_configuration);
        RCLCPP_INFO_STREAM(LOGGER,
                           "Configuring EGM for mechanical unit group " << group.name() << " on port " << egm_port);
      } catch (std::invalid_argument &e) {
        RCLCPP_FATAL_STREAM(LOGGER, "EGM port for mechanical unit group \"" << group.name()
                            << "\" not specified in hardware parameters");
        return CallbackReturn::ERROR;
      }
    }
    try {
      egm_manager_ = std::make_unique<abb::robot::EGMManager>(channel_configurations);
    } catch (std::runtime_error &e) {
      RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize EGM connection");
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn ABBSystemHardware::on_activate(const rclcpp_lifecycle::State & /* previous_state */) {
    size_t counter = 0;
    RCLCPP_INFO(LOGGER, "Connecting to robot...");
    while (rclcpp::ok() && ++counter < NUM_CONNECTION_TRIES) {
      // Wait for a message on any of the configured EGM channels.
      if (egm_manager_->waitForMessage(500)) {
        RCLCPP_INFO(LOGGER, "Connected to robot");
        break;
      }

      RCLCPP_INFO(LOGGER, "Not connected to robot...");
      if (counter == NUM_CONNECTION_TRIES) {
        RCLCPP_ERROR(LOGGER, "Failed to connect to robot");
        return CallbackReturn::ERROR;
      }
      rclcpp::sleep_for(500ms);
    }

    // The export transform is idempotent, so a missed read simply re-exports
    // the init-seeded values.
    egm_manager_->read(motion_data_);
    applyCouplingAndSetStates();
    if (has_egm_gpio_) {
      set_state("egm/connected", 1.0); // activation blocks until connected
    }

    // Commands start at the coupled states so the first write holds position.
    for (const auto &name: joint_names_) {
      set_command(name + "/" + hardware_interface::HW_IF_POSITION,
                  get_state(name + "/" + hardware_interface::HW_IF_POSITION));
      set_command(name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
    }

    RCLCPP_INFO(LOGGER, "ros2_control hardware interface was successfully started!");

    return CallbackReturn::SUCCESS;
  }

  return_type ABBSystemHardware::read(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Store previous connection state
    bool was_connected = egm_connected_;

    // Try to read from EGM. A fresh message reconnects immediately; the
    // lost verdict is debounced so a single late read (jitter, an overrun)
    // does not flip the connection state for one cycle.
    const bool fresh = egm_manager_->read(motion_data_);
    stale_reads_ = fresh ? 0 : std::min(stale_reads_ + 1, kDisconnectDebounceReads);
    egm_connected_ = fresh || (was_connected && stale_reads_ < kDisconnectDebounceReads);

    // Connection transitions are operational events, not debug noise: a
    // disconnect freezes the exported states and drops commands until the
    // stream returns.
    if (was_connected && !egm_connected_) {
      RCLCPP_WARN(LOGGER, "EGM connection lost - exported states frozen, commands dropped");
    }
    if (!was_connected && egm_connected_) {
      RCLCPP_INFO(LOGGER, "EGM connection restored - states live again");
    }
    if (has_egm_gpio_) {
      set_state("egm/connected", egm_connected_ ? 1.0 : 0.0);
    }

    // Only update values if connected.
    if (egm_connected_) {
      // Update force/torque measurements
      for (int i = 0; i < motion_data_.groups[0].egm_channel_data.input.mutable_measuredforce()->force_size(); i++) {
        urcl_ft_sensor_measurements_[i] = motion_data_.groups[0].egm_channel_data.input.mutable_measuredforce()->
            force(i);
      }
      for (const auto &sensor: info_.sensors) {
        for (uint j = 0; j < sensor.state_interfaces.size(); j++) {
          set_state(sensor.name + "/" + sensor.state_interfaces[j].name, urcl_ft_sensor_measurements_[j]);
        }
      }

      applyCouplingAndSetStates();
    }
    // else: keep last exported values when disconnected

    return return_type::OK;
  }

  void ABBSystemHardware::applyCouplingAndSetStates() {
    // The wire-side states stay raw; the J2-J3 coupling is folded into the
    // copied-out value only. This makes the transform idempotent: re-running
    // it on unchanged (stale) wire data recomputes the same exported values,
    // so a coupled value can never be re-coupled and wind up.
    size_t idx = 0;
    for (size_t g = 0; g < motion_data_.groups.size(); ++g) {
      auto &units = motion_data_.groups[g].units;
      for (size_t u = 0; u < units.size(); ++u) {
        auto &joints = units[u].joints;
        for (size_t k = 0; k < joints.size(); ++k) {
          double pos = joints[k].state.position;
          double vel = joints[k].state.velocity;
          if (j23_coupling_ && g == 0 && u == 0 && k == 2) {
            pos += J23_factor * joints[1].state.position;
            vel += J23_factor * joints[1].state.velocity;
          }
          set_state(joint_names_[idx] + "/" + hardware_interface::HW_IF_POSITION, pos);
          set_state(joint_names_[idx] + "/" + hardware_interface::HW_IF_VELOCITY, vel);
          ++idx;
        }
      }
    }
  }

  return_type ABBSystemHardware::write(const rclcpp::Time &time, const rclcpp::Duration &period) {
    // Only send commands if connected
    if (!egm_connected_) {
      return return_type::OK; // Skip writing when disconnected
    }

    // Copy the framework command interfaces onto the wire-side data, then
    // convert J3 to the raw (uncoupled) value the robot expects. The raw
    // value only ever exists in motion_data_, which controllers cannot see,
    // so there is no window where a concurrent reader of the command
    // interfaces can catch an uncoupled J3.
    size_t idx = 0;
    for (auto &group: motion_data_.groups) {
      for (auto &unit: group.units) {
        for (auto &joint: unit.joints) {
          joint.command.position = get_command(joint_names_[idx] + "/" + hardware_interface::HW_IF_POSITION);
          joint.command.velocity = get_command(joint_names_[idx] + "/" + hardware_interface::HW_IF_VELOCITY);
          ++idx;
        }
      }
    }

    if (j23_coupling_) {
      auto &joints = motion_data_.groups[0].units[0].joints;
      joints.at(2).command.position -= J23_factor * joints.at(1).command.position;
      joints.at(2).command.velocity -= J23_factor * joints.at(1).command.velocity;
    }

    // The EGM manager THROWS on NaN/out-of-range commands; an unhandled
    // exception here kills ros2_control_node. Drop the cycle instead - the
    // robot keeps its last reference and the offending controller is visible
    // in the log.
    try {
      egm_manager_->write(motion_data_);
    } catch (const std::exception &e) {
      static rclcpp::Clock throttle_clock;
      RCLCPP_ERROR_THROTTLE(LOGGER, throttle_clock, 1000,
                            "EGM rejected a joint command (%s) - cycle dropped", e.what());
    }

    return return_type::OK;
  }
} // namespace abb_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(abb_hardware_interface::ABBSystemHardware, hardware_interface::SystemInterface)
