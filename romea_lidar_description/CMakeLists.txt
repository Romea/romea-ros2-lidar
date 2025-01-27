cmake_minimum_required(VERSION 3.5)
project(romea_lidar_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_lidar_description.py)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

#if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  # ament_add_pytest_test(lidar_description_test test/lidar_description_test.py)
#  ament_add_pytest_test(view_ur_launch test/test_view_ur_launch.py)
#endif()

ament_package()
