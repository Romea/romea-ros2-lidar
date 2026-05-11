# romea_lidar_meta_bringup

## 1) Overview

romea_lidar_meta_bringup provides tools to describe and launch lidar sensor using a meta-description approach.

It allows defining lidar in a high-level YAML format and automatically generating consistent ROS2 artifacts such as:

* configuration files → used as generic ROS2 configuration inputs
* launch files → used to start lidar drivers and/or bridge for simulation
* URDF description files → used to load the lidar into simulators

This package is built on top of `romea_common_meta_bringup` and specializes it for lidar sensor integration. 

It also provides launch files that allow controlling the lidar both on a real robot and in simulation.

---

## 2) lidar meta-description concept

An `lidar meta-description` is a YAML file that defines an lidar sensor and how it should be integrated into a system. It centralizes:

* lidar identification (name, namespace)
* hardware configuration (manufacturer, model, version)
* kinematic attachment (parent link, pose)
* ROS2 launch description

---

### Example of meta-description

```yaml id="9lfc1h"
name: "lidar"
namespace: "ns"
launch:
  - include:
      file: "$(find-pkg-share romea_lidar_meta_bringup)/profile/sick_scan_xd_lms1xx.launch.py"
      arg:
        - name: ip
          value: "192.168.1.112"
        - name: port
          value: "2112"
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_lidar_meta_bringup)/profile/gz_bridge.launch.py
      arg:
        - name: container
          value: /gz_container 
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
configuration:
  manufacturer: sick
  model: lms
  version: "151"
  rate: 50 # hz
  azimut_resolution: 0.5 # degree
location:
  parent_link: "base_link"
  xyz: [1.0, 2.0, 3.0] # meters
  rpy: [4.0, 5.0, 6.0] # degrees
records:
  scan: true
  cloud: false
```

### Launch files Profiles

The `profile/` directory contains reusable ROS2 launch files dedicated to lidar bringup.

These launch profiles provide predefined setups for common execution contexts, such as:

* starting a real lidar driver, for example ouster_ros_driver.launch.py
* starting a simulation bridge, for example gz_bridge.launch.py
* reusing standardized bringup configurations across live and simulation modes

Each profile is intended to be included from the launch section of an lidar meta-description. This makes it possible to select the appropriate runtime behavior depending on the selected mode, while keeping the meta-description concise and consistent.

## 3) Scripts

`romea_lidar_meta_bringup` provides several scripts to generate ROS2 artifacts (configuration, launch and URDF files) from an lidar meta-description; the usage and resulting outputs are described below.

### Generate configuration file

```bash id="rq0ybi"
generate-lidar-configuration-file \
  meta_description_file_path:=path/to/lidar_meta_description.yaml \
  extended:=false
```

#### Example output

```yaml id="pgd8yu"
model: lms
version: 151
manufacturer: sick
rate: 50  # unit Hz
minimal_azimut_angle: -135.0  # unit °
maximal_azimut_angle: 135.0  # unit °
azimut_resolution: 0.5  # unit °
azimut_angle_std: 0.0  # unit °
minimal_range: 0.05  # unit m
maximal_range: 50.0  # unit m
range_std: 0.0  # unit m
samples: 541
parent_link: base_link
xyz: [1.0, 2.0, 3.0]  # unit m
rpy: [4.0, 5.0, 6.0]  # unit 
```
---

### Generate URDF Description

Generates the lidar URDF description from the meta-description.

```bash 
generate-lidar-urdf-description-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/lidar_meta_description.yaml \
  mode:=simulation_gazebo
```

The generated URDF description defines how the lidar is attached to the robot model. It includes the lidar link, the fixed joint between the parent link and the lidar frame, and, in simulation mode, the Gazebo sensor description.


#### Example output when using gazebo simulation

```xml
<link name="robot_lidar_link">
  ...
</link>

<joint name="robot_lidar_joint" type="fixed">
  <origin xyz="1.0 2.0 3.0"
          rpy="0.06981317007977318 0.08726646259971647 0.10471975511965977"/>
  <parent link="robot_base_link"/>
  <child link="robot_lidar_link"/>
</joint>

<gazebo reference="robot_lidar_link">
  <sensor name="robot_lidar" type="gpu_ray">
    <update_rate>50</update_rate>
    <topic>/robot/ns/lidar/scan</topic>
    <gz_frame_id>robot_lidar_link</gz_frame_id>
    ...
  </sensor>
</gazebo>
```

This URDF description can then be loaded into the robot description and used by Gazebo to publish simulated lidar measurements. 

It can also be directly concatenated with mobile base and other device URDF descriptions to build a complete robot model.

### Generate launch file

Generates a YAML ROS2 launch file from the meta-description.

```bash
generate-lidar-launch-file \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/lidar_meta_description.yaml
```

#### Example output

```yaml
launch:
- arg: {name: mode, default: live}
- group:
  - push-ros-namespace: {namespace: robot}
  - push-ros-namespace: {namespace: ns}
  - push-ros-namespace: {namespace: imu}
  - let: {name: model, value: lms}
  - let: {name: version, value: '151'}
  - let: {name: manufacturer, value: sick}
  - let: {name: rate, value: '50'}
  - let: {name: minimal_azimut_angle, value: '-135.0'}
  - let: {name: maximal_azimut_angle, value: '135.0'}
  - let: {name: azimut_resolution, value: '0.5'}
  - let: {name: azimut_angle_std, value: '0.0'}
  - let: {name: minimal_range, value: '0.05'}
  - let: {name: maximal_range, value: '50.0'}
  - let: {name: range_std, value: '0.0'}
  - let: {name: samples, value: '541'}
  - include:
      file: $(find-pkg-share romea_lidar_meta_bringup)/profile/sick_scan_xd_lms1xx.launch.py
      arg: [{name: ip, value: 192.168.1.112}, {name: port, value: '2112'}]
      if: $(eval "'$(var mode)' == 'live'")
  - include:
      file: $(find-pkg-share romea_lidar_meta_bringup)/profile/gz_bridge.launch.py
      arg: [{name: container, value: /gz_container}]
      if: $(eval "'$(var mode)' == 'simulation_gazebo'")
```

#### Notes

* the launch file is generated from the `launch` section of the meta-description
* namespaces are automatically constructed (`robot → device → lidar`)
* all configuration values are exposed as `let` variables
* the selected profiles are included at the end

This file can be used directly with ROS2 or generated dynamically using `lidar.launch.py`.

## 4) Usage

The package provides **two main launch files**:

* `lidar.launch.py` → for dynamic bringup (live or simulation mode)
* `simulation_test.launch.py` → for full simulation test

---

### Dynamic bringup

When using `lidar.launch.py`, the following steps are performed automatically:

```bash
ros2 launch romea_lidar_meta_bringup lidar.launch.py \
  mode:=live \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/lidar_meta_description.yaml
```

* generation of the URDF description
* generation of the lidar configuration file
* generation of the launch file
* execution of the generated launch file


#### Live or simulation mode

The `mode` parameter controls the behavior:

* `live` → starts the lidar driver and controllers
* `simulation_<simulator>` → starts simulation bridge

---

### Simulation test

For a complete simulation setup, use:

```bash
ros2 launch romea_lidar_meta_bringup simulation_test.launch.py \
  simulator_type:=gazebo \
  robot_namespace:=robot \
  meta_description_file_path:=path/to/lidar_meta_description.yaml
```

This launch file:

* starts the simulator
* generates and loads the URDF
* spawns the lidar in simulation
* calls `lidar.launch.py` to start the gazebo bridge

---
## 5) Supported lidars

Currently, the following lidar manufacturers and models are supported::


| Manufacturer | Model | Version |
|:------------:|:-----:|:-------:|
| sick | lms | 151 |
| sick | mrs | 1000 |
| sick | tim | 551 |
| ouster | os | 1 |
| robotsense | airy |  |

Details and specifications for each model can be found in the config directory of the `romea_lidar_description` package.


## 6) Supported lidar ROS2 Drivers

The package currently supports the following ROS2 lidar drivers:

* sick_can_xd
* ouster_ros

Dedicated launch profiles are provided in the profile/ directory to start each supported driver through a standardized interface.

These profiles can be included directly from the launch section of the lidar meta-description.

#### Example using sick_can_xd

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_lidar_meta_bringup)/profile/sick_scan_xd_lms1xx.launch.py"
      arg:
            arg: [{name: ip, value: 192.168.1.112}, {name: port, value: '2112'}]
        - name: ip
          value: 192.168.1.112
        - name: port
          value: 2112
```

#### Example using oster_ros

```yaml
launch:
  - include:
      file: "$(find-pkg-share romea_lidar_meta_bringup)/profile/ouster_ros_driver.launch.py"
      arg:
        - name: ip
          value: 192.168.1.112
        - name: port
          value: 0
```

Each launch profile is responsible for:

* starting the corresponding driver node
* configuring driver parameters
* applying standardized topic remappings

This abstraction ensures that all supported lidar drivers expose a consistent ROS2 interface independently of their internal implementation.
