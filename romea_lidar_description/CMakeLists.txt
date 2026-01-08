cmake_minimum_required(VERSION 3.5)
project(romea_lidar_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY config urdf meshes launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_lidar_description.py)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/../.flake8")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_sick_lsm1xx_description test/test_sick_lsm1xx_description.py)
  ament_add_pytest_test(test_sick_tim5xx_description test/test_sick_tim5xx_description.py)
  ament_add_pytest_test(test_sick_mrs1xxx_description test/test_sick_mrs1xxx_description.py)
  ament_add_pytest_test(test_ouster_os1_description test/test_ouster_os1_description.py)
  ament_add_pytest_test(test_lidar_urdf test/test_lidar_urdf.py)
endif()

ament_package()
