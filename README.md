# Multiantenna GNSS plugin for Gazebo

Computes true north heading based on two GPS sensors in Gazebo.

## Installation

Developed and tested with ROS eloquent.

* clone to your ros2 workspace
* rosdep install --from-path src -i
* colcon build
* source install/setup.bash

Make sure the package environment is sourced, so the library is on `LD_LIBRARY_PATH` when spawning a robot into a Gazebo world.

## Running

To run the test world with Gazebo.

```bash
# 1. build project to generate the test model
# 2. run test world
gazebo --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so test/worlds/gazebo_ros_multiantenna_gnss.world --ros-args --param publish_rate:=200.0
```

## Parameters

The following parameters can be passed as children on the sdf plugin element.

* `aux_sensor` (required): name of sensor to reference the heading to

### Publishers

* `gphdt`: measured heading `novatel_gps_msgs/msg/Gphdt`

### Example

```xml
<sensor>
    ...
    <plugin name="multiantenna" filename="libgazebo_ros_multiantenna_gnss.so">
        <ros>
          <namespace>vehicle0</namespace>
        </ros>
        <aux_antenna>aux_antenna_sensor_name</aux_antenna>
        <update_rate>1.0</update_rate>
        <frame_id>antenna_name</frame_id>
      </plugin>
      ...
</sensor>
```

## License and Copyright

Copyright [2020] AIT Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.