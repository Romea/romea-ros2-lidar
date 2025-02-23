cmake_minimum_required(VERSION 3.5)
project(romea_lidar_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_lidar_meta_bringup.py)

install(PROGRAMS
  scripts/urdf_description.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_lidar_meta_bringup.yaml test_lidar_meta_bringup.yaml)
  configure_file(test/test_sick_scan_xd_lms1xx_profile.yaml test_sick_scan_xd_lms1xx_profile.yaml)
  ament_add_pytest_test(test_lidar_driver_configuration test/test_lidar_driver_configuration.py)
  ament_add_pytest_test(test_lidar_meta_bringup test/test_lidar_meta_bringup.py)
  ament_add_pytest_test(test_urdf_description test/test_urdf_description.py)
endif()

ament_package()
