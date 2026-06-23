// Copyright 2024 OmniCore
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

#include <gtest/gtest.h>
#include "abb_hardware_interface/abb_hardware_interface.hpp"

static hardware_interface::ComponentInfo joint(const std::string& n,
    std::unordered_map<std::string,std::string> p = {}) {
  hardware_interface::ComponentInfo c; c.name = n; c.parameters = std::move(p); return c;
}

TEST(BuildDescription, SingleGroupWhenNoMechanicalUnitParam) {
  std::vector<hardware_interface::ComponentInfo> joints{
    joint("joint_1"), joint("joint_2"), joint("joint_3")};
  auto desc = abb_hardware_interface::ABBSystemHardware::buildDescriptionFromJoints(joints, {});
  ASSERT_EQ(desc.mechanical_units_groups_size(), 1);
  EXPECT_EQ(desc.mechanical_units_groups(0).name(), "");
  EXPECT_EQ(desc.mechanical_units_groups(0).robot().standardized_joints_size(), 3);
}

TEST(BuildDescription, TwoGroupsArmRobotPlusExternalSingle) {
  std::vector<hardware_interface::ComponentInfo> joints{
    joint("joint_1", {{"mechanical_unit","rob1"}}),
    joint("joint_2", {{"mechanical_unit","rob1"}}),
    joint("ext_joint_1", {{"mechanical_unit","extax"},{"mechanical_unit_type","single"}})};
  auto d = abb_hardware_interface::ABBSystemHardware::buildDescriptionFromJoints(joints, {});
  ASSERT_EQ(d.mechanical_units_groups_size(), 2);
  EXPECT_EQ(d.mechanical_units_groups(0).name(), "rob1");
  EXPECT_TRUE(d.mechanical_units_groups(0).has_robot());
  EXPECT_EQ(d.mechanical_units_groups(0).robot().standardized_joints_size(), 2);
  EXPECT_EQ(d.mechanical_units_groups(1).name(), "extax");
  EXPECT_FALSE(d.mechanical_units_groups(1).has_robot());
  ASSERT_EQ(d.mechanical_units_groups(1).mechanical_units_size(), 1);
  EXPECT_EQ(d.mechanical_units_groups(1).mechanical_units(0).type(), abb::robot::MechanicalUnit_Type_SINGLE);
  EXPECT_EQ(d.mechanical_units_groups(1).mechanical_units(0).standardized_joints(0).standardized_name(), "ext_joint_1");
}
