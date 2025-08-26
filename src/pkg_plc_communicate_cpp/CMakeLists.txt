cmake_minimum_required(VERSION 3.8)
project(pkg_plc_communicate_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pkg_beien_paint_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

add_executable(plc_communicate 
  src/pkg_plc_communicate_cpp/plc_communicate_node.cpp
  src/plc_communicate.cpp
)
target_include_directories(plc_communicate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pkg_plc_communicate_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBMODBUS_INCLUDE_DIRS}
)
target_compile_features(plc_communicate PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  plc_communicate
  "rclcpp"
  "rclcpp_action"
  "pkg_beien_paint_msgs"
  "std_msgs"
)

target_link_libraries(plc_communicate
  ${LIBMODBUS_LIBRARIES}
)

target_compile_options(plc_communicate PUBLIC ${LIBMODBUS_CFLAGS_OTHER})

install(TARGETS plc_communicate
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
