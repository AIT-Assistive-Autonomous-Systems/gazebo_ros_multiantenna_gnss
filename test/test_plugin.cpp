// Copyright 2020 AIT Austrian Institute of Technology GmbH
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

#include <gazebo/common/Time.hh>
#include <gazebo/test/ServerFixture.hh>
#include <memory>
#include <novatel_gps_msgs/msg/gphdt.hpp>
#include <rclcpp/rclcpp.hpp>

// floating-point tolerance for test comparisons
#define tol 10e-1

using namespace std::literals::chrono_literals; // NOLINT
using novatel_gps_msgs::msg::Gphdt;

class GazeboRosMultiantennaGnssTest : public gazebo::ServerFixture
{
};

TEST_F(GazeboRosMultiantennaGnssTest, Measure45deg)
{
  this->Load("test/worlds/gazebo_ros_multiantenna_gnss.world", true);

  auto world = gazebo::physics::get_world();
  ASSERT_NE(nullptr, world);

  auto main_gps = gazebo::sensors::SensorManager::Instance()->GetSensor("main_gps");
  ASSERT_NE(nullptr, main_gps);

  auto node = std::make_shared<rclcpp::Node>("gazebo_ros_multiantenna_gnss");
  ASSERT_NE(nullptr, node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  Gphdt::SharedPtr last_msg = nullptr;
  auto pub = node->create_subscription<Gphdt>(
    "gphdt", rclcpp::SensorDataQoS(), [&last_msg](Gphdt::SharedPtr msg) {
      last_msg = msg;
    });
  world->Step(21);
  executor.spin_once(210ms);

  ASSERT_NE(nullptr, last_msg);
  EXPECT_NEAR(45.0, last_msg->heading, tol);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
