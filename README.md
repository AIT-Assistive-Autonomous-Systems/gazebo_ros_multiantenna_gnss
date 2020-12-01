# Multiantenna GNSS plugin for Gazebo

Computes true north heading based on two GPS sensors in Gazebo.

## Installation

* clone to your workspace
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
* `frame_name`: frame id of the published message

To add simulated noise, configure noise on one or both of the sensors determining the positions.

Make sure the referenced antenna link is defined before the main antenna in the SDF, since it will be queried at initialization.

### Publishers

* `gphdt`: measured heading `novatel_gps_msgs/msg/Gphdt`

### Example

```xml
<sensor name="aux_gps" type="gps">
    <always_on>true</always_on>
    <update_rate>1.0</update_rate>
    <gps>
      <position_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>1e-4</mean>
            <stddev>1e-5</stddev>
          </noise>
        </horizontal>
        <vertical>
          <noise type="gaussian">
            <mean>1e-4</mean>
            <stddev>1e-5</stddev>
          </noise>
        </vertical>
      </position_sensing>
    </gps>
    ...
    <plugin name="multiantenna_gnss0" filename="libgazebo_ros_multiantenna_gnss.so">
        <ros>
          <namespace>vehicle0</namespace>
        </ros>
        <aux_sensor>aux_sensor_name</aux_sensor>
        <frame_name>antenna_frame</frame_name>
      </plugin>
      ...
</sensor>
```

## License and Copyright

Copyright 2020 AIT Austrian Institute of Technology GmbH

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.