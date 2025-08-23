cmake_minimum_required(VERSION 3.8)

project(modbus_test)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED true)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin/Debug)

add_executable(modbus_test 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/modbus_helper.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
  )

target_include_directories(modbus_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${LIBMODBUS_INCLUDE_DIRS}
)

target_link_libraries(modbus_test
  ${LIBMODBUS_LIBRARIES}
)