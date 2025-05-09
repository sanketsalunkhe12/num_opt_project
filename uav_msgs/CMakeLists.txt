cmake_minimum_required(VERSION 3.8)
project(uav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  std_msgs
  geometry_msgs
  nav_msgs
  rosidl_default_generators
)

set(msg_list
    "msg/PositionCmd.msg"
    "msg/ConsensusTraj.msg")

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_list}
    DEPENDENCIES ${dependencies})

ament_export_dependencies(${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
