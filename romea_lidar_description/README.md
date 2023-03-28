# 1 Overview #

This package contains the description of lidar sensors used in romea projects

# 2 Package organization #

This package is organized into subdirectories as follows:

  - urdf/ contains (xacro representations of) urdf descriptions of a lidar sensors.

  - config/ contains characteristic description of following lidar:

    - sick lms1xx family
    - sick tim5xx family
    - sick mrs100 lidar

  - python/ contains romea_lidar_description python module able to create LIDAR URDF description according their xacro representations and required parameters given by user
