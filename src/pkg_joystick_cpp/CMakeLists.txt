cmake_minimum_required(VERSION 3.8)
project(pkg_joystick_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pkg_beien_paint_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(HIDAPI REQUIRED hidapi-hidraw)

add_executable(joycon_left_pub 
  src/pkg_joystick_cpp/joycon_left_node.cpp
  src/joycon_left_pub.cpp
)
target_include_directories(joycon_left_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pkg_joystick_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joycon_left_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joycon_left_pub
  "rclcpp"
  "pkg_beien_paint_msgs"
  "rclcpp_action"
  "std_msgs"
)
target_link_libraries(joycon_left_pub ${HIDAPI_LIBRARIES})
install(TARGETS joycon_left_pub
  DESTINATION lib/${PROJECT_NAME})

add_executable(joycon_right_pub 
  src/pkg_joystick_cpp/joycon_right_node.cpp
  src/joycon_right_pub.cpp
)
target_include_directories(joycon_right_pub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pkg_joystick_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(joycon_right_pub PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  joycon_right_pub
  "rclcpp"
  "pkg_beien_paint_msgs"
  "rclcpp_action"
  "std_msgs"
)
target_link_libraries(joycon_right_pub ${HIDAPI_LIBRARIES})
install(TARGETS joycon_right_pub
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装参数文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
