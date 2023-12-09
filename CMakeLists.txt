cmake_minimum_required(VERSION 3.8)
project(crx_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_control REQUIRED)


install(DIRECTORY
  urdf
  meshes
  launch
  worlds
  config
  images
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(
    PROGRAMS
    scripts/test/fk_publisher.py
    scripts/perception_node.py
    scripts/ik_publisher.py
    scripts/inverse_kinematics.py
    DESTINATION lib/${PROJECT_NAME}
    )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
