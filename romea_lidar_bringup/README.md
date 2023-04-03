# 1) Overview #

The romea_lidar_bringup package provides  : 

 - launch files able to launch ros2 receiver drivers according a meta-description file provided by user (see next section for LIDAR meta-description file overview), only one driver is supported for the moment :

   - [sick_scan](https://github.com/SICKAG/sick_scan_xd)

   It is possible to launch a driver via command line : 

    ```console
    ros2 launch romea_lidar_bringup lidar_driver.launch.py robot_namespace:=robot meta_description_file_path:=/path_to_file/meta_description_file.yaml
    ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

 - a python module able to load and parse LIDAR meta-description file as well as to create URDF description of the LIDAR sensor according a given meta-description.

 - a ros2 python executable able to create LIDAR URDF description via command line according a given meta-description file  :

  ```console
  ros2 run romea_lidar_bringup urdf_description.py robot_namespace:robot meta_description_file_path:/path_to_file/meta_description_file.yaml > lidar.urdf`
  ```

   where :

   - *robot_namespace* is the name of the robot 
   - *meta_description_file_path* is the absolute path of meta-description file    

   This URDF  can be directly concatened with mobile base and other sensor URDFs to create a complete URDF description of the robot.  

   



# 2) LIDAR meta-description #

As seen below LIDAR meta-description file is a yaml file constituted by six items. The first item is the name of sensor defined by user. The second one is the configuration of ROS2 driver used to control LIDAR receiver (see section 4 for more explanations). The third item provides basics specifications of the LIDAR receiver and the fourth item specifies where the LIDAR is located on the robot, these informations will be used to create URDF description and by user to configure its algorithms.  Finally, the last item gives the topics to be recorded into the ROS bag during experiments or simulation. Thanks to remappings written into launch files, LIDAR topics are always the same names for each drivers or simulator plugins.       

Example :
```yaml
name: "lidar"  # name of the lidar
driver: #driver configuration
  pkg: "sick_scan" #ros2 driver package name
  ip: "192.168.1.112" #device ip
  port: "2112" #communication port
configuration: # lidar configuration
  type: sick  # lidar type
  model: lms151 # lidar model
  rate: 50 # hz (optional according lidar model)
  resolution: 0.5  # deg (optional according lidar model)
geometry: #geometry configuration
  parent_link: "base_link" # name of parent link where is located the LIDAR senor
  xyz: [2.02, 0.0, 0.34] # its position in meters
  rpy: [0.0, 0.0, 0.0] # its orienation in degrees
records: #record configuration
  scan: true  # scan topic will be recorded in bag
  cloud: false # cloud topic wil not be recorded in bag
```

# 4) Supported LIDAR models

Supported LIDAR receiver are listed in the following table :

|  type  |   model    |
| :----: | :--------: |
| sick   |  lmsxx     |
| sick   |  tim5xx    |

You can find specifications of each receiver in config directory of romea_driver_description package.

# 5) Supported LIDAE ROS2 drivers

Only [sick_scan](https://github.com/SICKAG/sick_scan_xd) is supported for the moment. In order to it, you can specify driver item in LIDAR meta-description file like this:

- Sick scan:

```yaml
  pkg: "sick_scan" #ros2 driver package name
  ip: "192.168.1.112"  # device IP
  port: "2112" # communication port
```

For each driver a python launch file with the name of the ROS2 package is provided in launch directory. When the meta-description is red by the main launch file called lidar_driver.launch.py the corresponding driver is automatically launched taking into account parameters define by user. Thanks to remapping defined inside each driver launch files, the data provided by drivers are always publish in the same topics called:

- scan(sensor_msgs/LaserScan)
- cloud(sensor_msgs/PointCloud2)
