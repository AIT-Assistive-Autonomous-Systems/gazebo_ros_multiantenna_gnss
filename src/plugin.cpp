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
#include <ignition/math/Angle.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/GpsSensor.hh>
#include <sdf/sdf.hh>

#include <memory>
#include <string>
#include <vector>

#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <novatel_gps_msgs/msg/gphdt.hpp>

#include "gazebo_ros_multiantenna_gnss/plugin.hpp"

namespace gazebo_plugins
{
using std::string;
using ignition::math::Angle;
using gazebo::common::Time;
using gazebo::sensors::GpsSensor;
using novatel_gps_msgs::msg::Gphdt;

class GazeboRosMultiantennaGnssPrivate
{
public:
  /// Callback to be called at every sensor update
  void OnUpdate();

  /// Pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Pointer to the main sensor
  gazebo::sensors::GpsSensorPtr sensor_;

  /// Pointer to the aux sensor
  gazebo::sensors::GpsSensorPtr aux_sensor_;

  // Message header frame_id
  string frame_id_;

  /// Publisher of GPS heading
  rclcpp::Publisher<Gphdt>::SharedPtr pub_gphdt_;

  /// Main gps sensor update event
  gazebo::event::ConnectionPtr sensor_update_event_;
};

GazeboRosMultiantennaGnss::GazeboRosMultiantennaGnss()
: impl_(std::make_unique<GazeboRosMultiantennaGnssPrivate>())
{
}

void GazeboRosMultiantennaGnss::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  impl_->sensor_ = std::dynamic_pointer_cast<GpsSensor>(_sensor);
  if (!impl_->sensor_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Parent is not a gps sensor. Exiting.");
    return;
  }

  impl_->pub_gphdt_ =
    impl_->ros_node_->create_publisher<Gphdt>("~/gphdt", rclcpp::SensorDataQoS());

  auto aux_sensor_param = _sdf->Get<std::string>("aux_sensor", "");
  if (!aux_sensor_param.second) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "'aux_sensor' parameter is required. Exiting.");
    return;
  }

  auto aux_sensor_name = aux_sensor_param.first;
  auto aux_sensor = gazebo::sensors::SensorManager::Instance()->GetSensor(aux_sensor_name);
  if (nullptr == aux_sensor) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "aux sensor '%s' not found. Exiting.", aux_sensor_name.c_str());
    return;
  }
  auto aux_sensor_gps = std::dynamic_pointer_cast<GpsSensor>(aux_sensor);
  if (nullptr == aux_sensor_gps) {
    RCLCPP_ERROR(
      impl_->ros_node_->get_logger(),
      "aux sensor '%s' is not a gps sensor. Exiting.", aux_sensor_name.c_str());
    return;
  }
  impl_->aux_sensor_ = aux_sensor_gps;
  impl_->frame_id_ = gazebo_ros::SensorFrameID(*_sensor, *_sdf);

  impl_->sensor_update_event_ = impl_->sensor_->ConnectUpdated(
    std::bind(&GazeboRosMultiantennaGnssPrivate::OnUpdate, impl_.get()));
}

inline Angle Bearing(double lat1, double lon1, double lat2, double lon2)
{
  double d_lon = lon2 - lon1;
  double y = sin(d_lon) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(d_lon);
  double theta = atan2(y, x);
  return Angle(theta + M_PI);
}

void GazeboRosMultiantennaGnssPrivate::OnUpdate()
{
  auto msg = std::make_unique<Gphdt>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(
    sensor_->LastUpdateTime());

  double main_lat = sensor_->Latitude().Radian();
  double main_lon = sensor_->Longitude().Radian();

  double aux_lat = aux_sensor_->Latitude().Radian();
  double aux_lon = aux_sensor_->Longitude().Radian();

  msg->message_id = "GPHDT";
  msg->heading = Bearing(main_lat, main_lon, aux_lat, aux_lon).Degree();
  msg->t = "T";
  pub_gphdt_->publish(std::move(msg));
}

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosMultiantennaGnss)
}  // namespace gazebo_plugins
