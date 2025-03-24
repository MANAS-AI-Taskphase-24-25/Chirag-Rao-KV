cmake_minimum_required(VERSION 3.8)
project(path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_executable(map_subscriber src/path_finder.cpp)
# ament_target_dependencies(map_processor rclcpp nav_msgs OpenCV yaml-cpp)


ament_target_dependencies(map_subscriber rclcpp nav_msgs geometry_msgs)

install(TARGETS map_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  maps
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
