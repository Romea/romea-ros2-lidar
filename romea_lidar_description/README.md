# romea_lidar_description

## Overview

`romea_lidar_description` provides URDF descriptions and configuration utilities for lidar sensors using a **specification-based approach**.

It allows defining lidar characteristics and generating consistent ROS2 artifacts using the provided Python module, such as:

* configuration files
* URDF description files

This package is designed to be used together with `romea_lidar_meta_bringup`.

---

## Lidar description concept

A lidar is described using two inputs:

* **lidar description**: defines hardware characteristics (manufacturer, model, version, rate)
* **lidar location**: defines how the lidar is attached to the robot (parent link, pose)

These inputs are combined with lidar specifications to build a complete configuration used to generate ROS2 artifacts.

---

### Example

```yaml
lidar_description:
  manufacturer: sick
  model: lms
  version: "151"
  rate: 25

lidar_location:
  parent_link: base_link
  xyz: [-0.5, 0.0, 1.0] #m
  rpy: [0.0, 0.0, 0.0] #°
```

---

### Notes

Lidar specifications are defined in files located in the `config/` directory and follow the pattern `<manufacturer>_<model>_<version>_specifications.yaml` (e.g. `sick_lms_1xx_specifications.yaml`). These specification files provide default values such as scan rate, angular resolution, measurement ranges, field of view, noise characteristics which can be overridden by user-defined values in `lidar_description`. 

The `xyz` and `rpy` fields define the pose of the lidar relative to the parent link. The `xyz` values specify the translation (in meters), while `rpy` defines the orientation using roll, pitch, and yaw angles expressed in degrees for readability and ease of definition by users. These angles are automatically converted to radians internally by the scripts to comply with ROS and URDF conventions. Together, they describe how the lidar is positioned and oriented within the robot frame.

---

## Python API

The package provides utilities to generate configuration, controllers configuration and URDF descriptions from an lidar description.

---

### get_complete_configuration

Builds a complete lidar configuration by combining:

* lidar description
* lidar location
* lidar specifications

---

### generate_configuration_file_str

Generates the lidar configuration file as a YAML string from the configuration returned by get_complete_configuration.

---

### generate_urdf_description_str

Generates the lidar URDF description as a string, ready to be written to a URDF file. The URDF description is generated from the complete configuration returned by `get_complete_configuration` and from an associated geometry configuration file. The generated URDF is used by slidarlators and visualization tools.

---

## Example

```python
from romea_lidar_description import (
    get_complete_configuration,
    generate_configuration_file_str,
    generate_urdf_description_str,
)

lidar_name = "lidar"

lidar_description = {
  manufacturer: sick,
  model: lms,
  version: "151",
  rate: 25,
}

lidar_location = {
    "parent_link": "base_link",
    "xyz": [-0.5, 0.0, 1.0],
    "rpy": [0.0, 0.0, 0.0]
}

prefix = "robot_"
ros_namespace = "/robot/lidar"
mode = "live"

configuration = get_complete_configuration(
    lidar_name,
    lidar_description,
    lidar_location,
)

configuration_yaml = generate_configuration_file_str(configuration)

urdf_description = generate_urdf_description_str(
    prefix,
    mode,
    lidar_name,
    lidar_description,
    lidar_location,
    ros_namespace,
)
```

The returned values can be written to files if needed.

---

## Usage

This package is typically used together with:

* `romea_lidar_meta_bringup` → generates launch files and handles bringup

---

## Supported lidars

Currently, the package supports the following lidar manufacturers and models:


| Manufacturer | Model | Version |
|:------------:|:-----:|:-------:|
| sick | lms | 151 |
| sick | mrs | 1000 |
| sick | tim | 551 |
| ouster | os | 1 |
| robotsense | airy |  |

Support for additional lidar models may be added in future releases.

! ajouter une section qui décrit comment on ajoute un nouveau Lidar