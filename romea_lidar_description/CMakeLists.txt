cmake_minimum_required(VERSION 3.5)
project(romea_lidar_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config urdf
  DESTINATION share/${PROJECT_NAME}
)

# ament_python_install_module(python/romea_lidar_description.py)

ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_sick_lsm1xx_description test/test_sick_lsm1xx_description.py)
  ament_add_pytest_test(test_sick_tim5xx_description test/test_sick_tim5xx_description.py)
endif()

ament_package()
